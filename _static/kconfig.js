/**
 * kconfig.js – interactive enhancements for the Kconfig reference page.
 *
 * Runs only when the #configuration-options-reference section is present on
 * the page (i.e. the generated kconfig-reference.html).
 *
 * The options page is presented as a single flat, searchable list of cards
 * (the generated menu tree is flattened away). Enhancements:
 *
 *  1. DOM classification: tag each CONFIG_* section as a .kconfig-option card
 *     and turn its "Found in:" line into .kconfig-menupath breadcrumb chips.
 *  2. A one-time in-memory search index (name / prompt / help text).
 *  3. Flatten the menu tree into one plain list of cards.
 *  4. A sticky search box ("/" to focus, Esc to clear): multi-word AND
 *     matching over name/prompt/help, with results reordered by relevance
 *     (name matches, especially prefixes, float to the top).
 *  5. Deep-link support: #CONFIG_FOO reveals and scrolls to the card.
 *  6. :menuitem: hover previews that show option details and menu path.
 *
 * To avoid a flash of the un-flattened menu tree, the script (loaded in
 * <head>) tags <html> with `kconfig-js` before first paint; kconfig.css uses
 * that to hide the raw tree and show a loading indicator until init() removes
 * the tag. No-JS users never get the tag and see the full server-rendered page.
 */

(function () {
  'use strict';

  // This script is loaded synchronously in <head>, i.e. before the body (and
  // the huge server-rendered menu tree) is painted. Flag the document right
  // away so kconfig.css can hide that raw tree and show a loading indicator
  // until init() has flattened it into the searchable card list. The flag is
  // removed again in init(); no-JS users never get it and keep the full
  // server-rendered content.
  document.documentElement.classList.add('kconfig-js');

  var pageRoot;
  var KCONFIG_HOVER_DELAY_MS = 280;
  var KCONFIG_HOVER_GRACE_MS = 80;
  var kconfigPageCache = {};
  var kconfigTooltip = null;
  var kconfigTooltipHideTimer = null;
  var kconfigTooltipAnchor = null;
  var kconfigTooltipHovered = false;
  var kconfigTooltipRequest = 0;

  /* -------------------------------------------------------------------------
     1. DOM classification
     ------------------------------------------------------------------------- */

  function classifyOptions() {
    var sections = pageRoot.querySelectorAll('section[id^="config-"], div.section[id^="config-"]');
    sections.forEach(function (sec) {
      sec.classList.add('kconfig-option');
      convertFoundIn(sec);
    });
  }

  /**
   * Replace the generated "Found in:" paragraph (an <em> label followed by
   * <a> links separated by " > ") with .kconfig-menupath chip markup.
   */
  function convertFoundIn(optionSection) {
    var paras = optionSection.querySelectorAll('p');
    paras.forEach(function (p) {
      var em = p.querySelector('em');
      if (!em || !/found\s+in/i.test(em.textContent)) { return; }

      var container = document.createElement('span');
      container.className = 'kconfig-menupath';

      var links = [];
      p.childNodes.forEach(function (child) {
        if (child.nodeType === Node.ELEMENT_NODE && child.tagName === 'A') {
          links.push(child.cloneNode(true));
        }
      });

      links.forEach(function (a, i) {
        if (i > 0) {
          var sep = document.createElement('span');
          sep.className = 'kconfig-menupath-sep';
          sep.textContent = '\u2023'; // ‣
          container.appendChild(sep);
        }
        var chip = document.createElement('span');
        chip.className = 'kconfig-menupath-chip';
        a.classList.add('kconfig-menupath-link');
        chip.appendChild(a);
        container.appendChild(chip);
      });

      if (links.length > 0) {
        p.innerHTML = '';
        var label = document.createElement('em');
        label.textContent = 'Found in: ';
        p.appendChild(label);
        p.appendChild(container);
      }
    });
  }

  /* -------------------------------------------------------------------------
     Small DOM helpers
     ------------------------------------------------------------------------- */

  function isSectionEl(el) {
    return el && (el.tagName === 'SECTION' ||
      (el.tagName === 'DIV' && el.classList.contains('section')));
  }

  function directChildHeading(sec) {
    var n = sec.firstElementChild;
    while (n) {
      if (/^H[1-6]$/.test(n.tagName)) { return n; }
      n = n.nextElementSibling;
    }
    return null;
  }

  /** Heading text without the headerlink (¶). */
  function headingText(heading) {
    if (!heading) { return ''; }
    var t = '';
    Array.prototype.forEach.call(heading.childNodes, function (n) {
      if (n.nodeType === Node.TEXT_NODE) {
        t += n.textContent;
      } else if (n.nodeType === Node.ELEMENT_NODE &&
                 !n.classList.contains('headerlink')) {
        t += n.textContent;
      }
    });
    return t.trim();
  }

  /* -------------------------------------------------------------------------
     2. Search index (built once)
     ------------------------------------------------------------------------- */

  // [{ el, name, nameLC, promptLC, helpLC }]
  var allOptions = [];

  function buildIndex() {
    var sections = pageRoot.querySelectorAll(
      'section.kconfig-option, div.section.kconfig-option'
    );
    Array.prototype.forEach.call(sections, function (sec) {
      var heading = directChildHeading(sec);
      var name = headingText(heading) || sec.id || '';

      var prompt = '';
      var p = heading ? heading.nextElementSibling : null;
      while (p) {
        if (p.tagName === 'P') {
          var em = p.querySelector('em');
          if (!(em && /found\s+in/i.test(em.textContent))) {
            prompt = p.textContent.trim();
            break;
          }
        }
        p = p.nextElementSibling;
      }

      allOptions.push({
        el: sec,
        name: name,
        nameLC: name.toLowerCase(),
        promptLC: prompt.toLowerCase(),
        helpLC: sec.textContent.toLowerCase()
      });
    });
  }

  /* -------------------------------------------------------------------------
     3. Flatten the menu tree into a plain list of cards
     ------------------------------------------------------------------------- */

  // Flat list children (cards + anchor stubs) in original document order; used
  // by restoreOrder() to undo the relevance reshuffle when a search is cleared.
  var allNodes = [];

  function flattenToCardList() {
    var list = document.createElement('div');
    list.className = 'kconfig-card-list';
    var seen = {};

    function isCard(el) {
      return el.classList && el.classList.contains('kconfig-option');
    }

    // Walk pageRoot in document order. Option cards are moved as-is (their own
    // anchors travel inside them). For every *other* element that carries an id
    // -- menu sections and :ref: target spans -- emit a zero-height anchor stub
    // at the same position, so deep links from the sidebar TOC and "Found in:"
    // breadcrumbs still resolve and land just before that menu's first option.
    function walk(node) {
      var child = node.firstElementChild;
      while (child) {
        var next = child.nextElementSibling;  // capture before any move
        if (isCard(child)) {
          list.appendChild(child);
          // CONFIG sections can be nested when the generated RST reaches its
          // maximum heading depth. Move those descendants out as cards too.
          walk(child);
        } else {
          if (child.id && !seen[child.id]) {
            seen[child.id] = true;
            var stub = document.createElement('span');
            stub.id = child.id;
            stub.className = 'kconfig-anchor';
            child.removeAttribute('id');  // avoid a duplicate id in the document
            list.appendChild(stub);
          }
          walk(child);
        }
        child = next;
      }
    }
    walk(pageRoot);

    // Drop the now empty, id-stripped menu sections left under pageRoot.
    var leftovers = [];
    var c = pageRoot.firstElementChild;
    while (c) {
      if (c !== list && isSectionEl(c)) { leftovers.push(c); }
      c = c.nextElementSibling;
    }
    leftovers.forEach(function (m) { m.parentNode.removeChild(m); });

    pageRoot.appendChild(list);
    allNodes = Array.prototype.slice.call(list.children);
    return list;
  }

  /* -------------------------------------------------------------------------
     4. Search box + in-place filtering
     ------------------------------------------------------------------------- */

  var searchInput = null;
  var searchCount = null;
  var cardList = null;

  function injectSearchBar(listEl) {
    cardList = listEl;
    var bar = document.createElement('div');
    bar.className = 'kconfig-filter-bar';

    searchInput = document.createElement('input');
    searchInput.type = 'search';
    searchInput.className = 'kconfig-filter';
    searchInput.placeholder = 'Search options by name, title, or help text…  (press /)';
    searchInput.setAttribute('aria-label', 'Search Kconfig options');

    searchCount = document.createElement('span');
    searchCount.className = 'kconfig-filter-count';

    bar.appendChild(searchInput);
    bar.appendChild(searchCount);
    pageRoot.insertBefore(bar, listEl);

    setCount(allOptions.length);

    var debounce = null;
    searchInput.addEventListener('input', function () {
      clearTimeout(debounce);
      debounce = setTimeout(applyFilter, 150);
    });
    searchInput.addEventListener('keydown', function (ev) {
      if (ev.key === 'Escape') { searchInput.value = ''; applyFilter(); }
    });
  }

  function setCount(n, total) {
    if (!searchCount) { return; }
    searchCount.textContent = (total === undefined)
      ? n + ' options'
      : n + ' of ' + total + ' options';
  }

  /**
   * Score a card against the query terms. Returns -1 if any term is missing
   * (multi-word AND); otherwise a higher score means a more relevant match,
   * weighted so that name matches (especially prefixes) float to the top.
   */
  function scoreCard(o, terms) {
    var score = 0;
    for (var i = 0; i < terms.length; i++) {
      var t = terms[i];
      var inName = o.nameLC.indexOf(t);
      if (inName !== -1) {
        score += (inName === 0) ? 100 : 50;
      } else if (o.promptLC.indexOf(t) !== -1) {
        score += 10;
      } else if (o.helpLC.indexOf(t) !== -1) {
        score += 1;
      } else {
        return -1;  // this term matched nothing -> AND fails
      }
    }
    return score;
  }

  /** Re-append cards and anchor stubs in their original (build) order. */
  function restoreOrder() {
    var frag = document.createDocumentFragment();
    allNodes.forEach(function (n) { frag.appendChild(n); });
    cardList.appendChild(frag);
  }

  function applyFilter() {
    var q = (searchInput.value || '').trim().toLowerCase();
    var terms = q ? q.split(/\s+/) : [];

    if (terms.length === 0) {
      allOptions.forEach(function (o) { o.el.classList.remove('kconfig-option-hidden'); });
      restoreOrder();
      setCount(allOptions.length);
      return;
    }

    var matched = [];
    allOptions.forEach(function (o) {
      var s = scoreCard(o, terms);
      if (s < 0) {
        o.el.classList.add('kconfig-option-hidden');
      } else {
        o.el.classList.remove('kconfig-option-hidden');
        matched.push({ o: o, s: s });
      }
    });

    // Most relevant first; ties broken alphabetically by option name.
    matched.sort(function (a, b) {
      if (b.s !== a.s) { return b.s - a.s; }
      return a.o.nameLC < b.o.nameLC ? -1 : (a.o.nameLC > b.o.nameLC ? 1 : 0);
    });

    // Move matched cards (in ranked order) after the hidden ones, so the
    // visible sequence reflects relevance. Hidden cards stay put but invisible.
    var frag = document.createDocumentFragment();
    matched.forEach(function (m) { frag.appendChild(m.o.el); });
    cardList.appendChild(frag);

    setCount(matched.length, allOptions.length);
  }

  /* -------------------------------------------------------------------------
     5. Deep-link support
     ------------------------------------------------------------------------- */

  /**
   * Always clear an active search, then smooth-scroll to the target. Scrolling
   * waits two animation frames so it happens after the filter clear has
   * reflowed/reordered the list (avoids a jump to a stale position).
   */
  function revealAndScroll(target) {
    if (searchInput && searchInput.value) {
      searchInput.value = '';
      applyFilter();
    }
    target.classList.remove('kconfig-option-hidden');
    requestAnimationFrame(function () {
      requestAnimationFrame(function () {
        target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      });
    });
  }

  function scrollToHash(anchor) {
    if (!anchor) { return; }
    var target = document.getElementById(anchor);
    if (!target) { return; }
    revealAndScroll(target);
  }

  /**
   * Intercept clicks on in-page anchor links (breadcrumb chips, header
   * permalinks, help-text cross-refs, sidebar TOC). We cancel the browser's
   * native jump -- which would otherwise land on a filtered-out (display:none)
   * card or a zero-height stub before we clear the filter -- and handle the
   * reveal+scroll ourselves. Bound on document so sidebar links are covered too.
   */
  function onAnchorClick(ev) {
    if (ev.defaultPrevented || ev.button !== 0 ||
        ev.ctrlKey || ev.metaKey || ev.shiftKey || ev.altKey) {
      return;
    }
    var a = ev.target && ev.target.closest ? ev.target.closest('a[href]') : null;
    if (!a) { return; }
    // Only same-page fragment links ("#foo"), not "../other.html#foo".
    if (a.getAttribute('href').charAt(0) !== '#') { return; }
    var id = decodeURIComponent((a.hash || '').slice(1));
    if (!id) { return; }
    var target = document.getElementById(id);
    if (!target) { return; }  // unknown anchor: leave default behavior

    ev.preventDefault();
    if (window.history && history.pushState) {
      history.pushState(null, '', a.hash);  // keep URL shareable, no native jump
    }
    revealAndScroll(target);
  }

  /* -------------------------------------------------------------------------
     6. :menuitem: hover tooltips
     ------------------------------------------------------------------------- */

  function getOrCreateKconfigTooltip() {
    if (kconfigTooltip) { return kconfigTooltip; }

    kconfigTooltip = document.createElement('div');
    kconfigTooltip.className = 'kconfig-hover-tooltip';
    kconfigTooltip.setAttribute('role', 'dialog');
    kconfigTooltip.setAttribute('aria-label', 'Kconfig option details');
    document.body.appendChild(kconfigTooltip);

    kconfigTooltip.addEventListener('mouseenter', function () {
      kconfigTooltipHovered = true;
      clearTimeout(kconfigTooltipHideTimer);
    });
    kconfigTooltip.addEventListener('mouseleave', function () {
      kconfigTooltipHovered = false;
      scheduleKconfigTooltipHide();
    });

    return kconfigTooltip;
  }

  function hideKconfigTooltip() {
    if (!kconfigTooltip) { return; }
    kconfigTooltip.classList.remove('visible');
    kconfigTooltip.classList.remove('loading');
    kconfigTooltipAnchor = null;
    kconfigTooltipHovered = false;
    kconfigTooltipRequest += 1;
  }

  function scheduleKconfigTooltipHide() {
    clearTimeout(kconfigTooltipHideTimer);
    kconfigTooltipHideTimer = setTimeout(function () {
      if (!kconfigTooltipAnchor && !kconfigTooltipHovered) {
        hideKconfigTooltip();
      }
    }, KCONFIG_HOVER_GRACE_MS);
  }

  function positionKconfigTooltip(el, anchor) {
    var rect = anchor.getBoundingClientRect();
    var vw = document.documentElement.clientWidth;
    var vh = document.documentElement.clientHeight;
    var tipWidth = el.offsetWidth || Math.min(640, vw - 24);
    var left = rect.left;

    if (left + tipWidth > vw - 12) { left = vw - tipWidth - 12; }
    if (left < 12) { left = 12; }

    var tipHeight = el.offsetHeight || Math.min(460, vh * 0.65);
    var topBelow = rect.bottom + 6;
    var topAbove = rect.top - tipHeight - 6;
    var belowFits = topBelow + tipHeight <= vh - 12;
    var aboveFits = topAbove >= 12;
    var top;

    if (belowFits) {
      top = topBelow;
      el.setAttribute('data-placement', 'below');
    } else if (aboveFits) {
      top = topAbove;
      el.setAttribute('data-placement', 'above');
    } else {
      top = Math.max(12, Math.min(topBelow, vh - tipHeight - 12));
      el.setAttribute('data-placement', 'floating');
    }

    el.style.left = left + 'px';
    el.style.top = top + 'px';
  }

  function resolveTooltipLinks(root, baseUrl) {
    if (!baseUrl) { return; }
    root.querySelectorAll('a[href]').forEach(function (a) {
      var href = a.getAttribute('href');
      if (!href) { return; }
      try { a.setAttribute('href', new URL(href, baseUrl).href); } catch (e) {}
    });
  }

  function stripTooltipHeaderLinks(root) {
    root.querySelectorAll('a.headerlink').forEach(function (a) { a.remove(); });
  }

  function extractKconfigFragment(doc, anchorId, baseUrl) {
    var target = doc.getElementById(anchorId);
    var card;

    if (!target) { return null; }
    if (target.classList && target.classList.contains('kconfig-option')) {
      card = target;
    } else if (target.closest) {
      card = target.closest('.kconfig-option');
    }

    if (!card && isSectionEl(target)) {
      card = target;
    }
    if (!card && target.closest) {
      card = target.closest('section, div.section');
    }
    if (!card) { return null; }

    var clone = card.cloneNode(true);
    clone.classList.add('kconfig-option', 'kconfig-tooltip-card');
    clone.classList.remove('kconfig-option-hidden');
    clone.querySelectorAll(
      'section[id^="config-"], div.section[id^="config-"]'
    ).forEach(function (section) {
      section.remove();
    });
    stripTooltipHeaderLinks(clone);
    convertFoundIn(clone);
    resolveTooltipLinks(clone, baseUrl);

    return clone;
  }

  function isKconfigTooltipRequestActive(anchor, requestId) {
    return requestId === kconfigTooltipRequest &&
      (kconfigTooltipAnchor === anchor || kconfigTooltipHovered);
  }

  function showKconfigTooltip(anchor, frag, requestId) {
    if (!isKconfigTooltipRequestActive(anchor, requestId)) { return; }

    var tooltip = getOrCreateKconfigTooltip();
    tooltip.innerHTML = '';
    tooltip.classList.remove('loading');

    var wrapper = document.createElement('div');
    wrapper.className = 'rst-content';
    wrapper.appendChild(frag);
    tooltip.appendChild(wrapper);

    tooltip.style.top = '-9999px';
    tooltip.classList.add('visible');
    positionKconfigTooltip(tooltip, anchor);
  }

  function showKconfigTooltipLoading(anchor, requestId) {
    if (!isKconfigTooltipRequestActive(anchor, requestId)) { return; }

    var tooltip = getOrCreateKconfigTooltip();
    tooltip.innerHTML = '<span class="kconfig-tooltip-spinner" aria-hidden="true"></span>' +
      '<span>Loading option details\u2026</span>';
    tooltip.classList.add('loading', 'visible');
    tooltip.style.top = '-9999px';
    positionKconfigTooltip(tooltip, anchor);
  }

  function loadKconfigTooltip(anchor, href, requestId) {
    var hashIdx = href.indexOf('#');
    var anchorId = hashIdx >= 0 ? href.slice(hashIdx + 1) : '';
    var pageUrl;
    var currentPageUrl;
    var frag;

    if (!anchorId) { return; }

    if (hashIdx === 0) {
      frag = extractKconfigFragment(document, anchorId, window.location.href);
      if (frag) { showKconfigTooltip(anchor, frag, requestId); }
      return;
    }

    try {
      pageUrl = new URL(href.slice(0, hashIdx), window.location.href).href;
    } catch (e) {
      return;
    }
    currentPageUrl = window.location.href.split('#')[0];

    if (pageUrl === currentPageUrl) {
      frag = extractKconfigFragment(document, anchorId, pageUrl);
      if (frag) { showKconfigTooltip(anchor, frag, requestId); }
      return;
    }

    if (kconfigPageCache[pageUrl]) {
      frag = extractKconfigFragment(kconfigPageCache[pageUrl], anchorId, pageUrl);
      if (frag) { showKconfigTooltip(anchor, frag, requestId); }
      return;
    }

    showKconfigTooltipLoading(anchor, requestId);
    fetch(pageUrl).then(function (resp) {
      if (!resp.ok) { return null; }
      return resp.text();
    }).then(function (html) {
      if (!html) {
        if (requestId === kconfigTooltipRequest) { hideKconfigTooltip(); }
        return;
      }
      kconfigPageCache[pageUrl] = new DOMParser().parseFromString(html, 'text/html');
      frag = extractKconfigFragment(kconfigPageCache[pageUrl], anchorId, pageUrl);
      if (frag) { showKconfigTooltip(anchor, frag, requestId); }
    }).catch(function () {
      // Ignore fetch errors; the link still works as normal navigation.
      if (requestId === kconfigTooltipRequest) { hideKconfigTooltip(); }
    });
  }

  function attachKconfigHover(anchor) {
    var showTimer = null;
    anchor.removeAttribute('title');

    anchor.addEventListener('mouseenter', function () {
      kconfigTooltipAnchor = anchor;
      kconfigTooltipRequest += 1;
      var requestId = kconfigTooltipRequest;
      clearTimeout(kconfigTooltipHideTimer);
      clearTimeout(showTimer);
      showTimer = setTimeout(function () {
        loadKconfigTooltip(anchor, anchor.getAttribute('href') || '', requestId);
      }, KCONFIG_HOVER_DELAY_MS);
    });

    anchor.addEventListener('mouseleave', function () {
      clearTimeout(showTimer);
      if (kconfigTooltipAnchor === anchor) {
        kconfigTooltipAnchor = null;
      }
      scheduleKconfigTooltipHide();
    });

    anchor.addEventListener('focus', function () {
      kconfigTooltipAnchor = anchor;
      kconfigTooltipRequest += 1;
      clearTimeout(kconfigTooltipHideTimer);
      loadKconfigTooltip(
        anchor,
        anchor.getAttribute('href') || '',
        kconfigTooltipRequest
      );
    });

    anchor.addEventListener('blur', function () {
      if (kconfigTooltipAnchor === anchor) {
        kconfigTooltipAnchor = null;
      }
      scheduleKconfigTooltipHide();
    });
  }

  function initMenuitemHover() {
    document.querySelectorAll('.kconfig-menuitem-chip > a[href*="#"]').forEach(function (a) {
      attachKconfigHover(a);
    });
    window.addEventListener('scroll', hideKconfigTooltip, {passive: true});
    window.addEventListener('resize', hideKconfigTooltip, {passive: true});
    document.addEventListener('keydown', function (ev) {
      if (ev.key === 'Escape') { hideKconfigTooltip(); }
    });
  }

  /* -------------------------------------------------------------------------
     Boot
     ------------------------------------------------------------------------- */

  /** Reveal the page once enhancement is done (or on any non-kconfig page). */
  function reveal() {
    document.documentElement.classList.remove('kconfig-js');
  }

  function init() {
    pageRoot = document.getElementById('configuration-options-reference');

    try {
      initMenuitemHover();
      if (!pageRoot) { return; }

      classifyOptions();
      var listEl = flattenToCardList();
      buildIndex();
      injectSearchBar(listEl);

      // Press "/" to focus the search box (unless already typing in a field).
      document.addEventListener('keydown', function (ev) {
        if (ev.key !== '/' || ev.ctrlKey || ev.metaKey || ev.altKey) { return; }
        var t = ev.target;
        var tag = t && t.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || (t && t.isContentEditable)) { return; }
        ev.preventDefault();
        searchInput.focus();
      });

      // Intercept in-page anchor clicks (also covers back/forward via hashchange).
      document.addEventListener('click', onAnchorClick);

      if (location.hash) { scrollToHash(location.hash.slice(1)); }
      window.addEventListener('hashchange', function () {
        scrollToHash(location.hash.slice(1));
      });
    } finally {
      // Always reveal: on success the flat searchable list appears at once; if
      // any step above threw, fall back to showing the content un-enhanced
      // rather than leaving the page hidden.
      reveal();
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
}());
