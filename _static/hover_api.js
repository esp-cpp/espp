(function () {
    'use strict';

    const HOVER_DELAY_MS = 280;
    const GRACE_MS = 80;

    const pageCache = new Map();  // keyed by page URL (without fragment)

    // Stack of tooltip levels: each entry is { el, hideTimer, active }
    const stack = [];

    function getOrCreateLevel(level) {
        while (stack.length <= level) {
            const entry = { el: null, hideTimer: null, active: false };
            const el = document.createElement('div');
            el.className = 'esp-hover-tooltip';
            document.body.appendChild(el);
            entry.el = el;
            el.style.setProperty('--hover-level', stack.length);

            const lvl = stack.length;
            el.addEventListener('mouseenter', function () {
                entry.active = true;
                clearTimeout(entry.hideTimer);
                // Keep all ancestor tooltips alive while mouse is inside a child
                for (let i = 0; i < lvl; i++) {
                    clearTimeout(stack[i].hideTimer);
                }
            });
            el.addEventListener('mouseleave', function () {
                entry.active = false;
                scheduleHide(lvl);
            });

            stack.push(entry);
        }
        return stack[level];
    }

    function hideFrom(level) {
        for (let i = level; i < stack.length; i++) {
            stack[i].el.classList.remove('visible');
            stack[i].active = false;
        }
    }

    function scheduleHide(level) {
        if (level >= stack.length) return;
        const entry = stack[level];
        clearTimeout(entry.hideTimer);
        entry.hideTimer = setTimeout(function () {
            if (!entry.active) hideFrom(level);
        }, GRACE_MS);
    }

    function positionTooltip(el, anchor) {
        const rect = anchor.getBoundingClientRect();
        const vw = document.documentElement.clientWidth;
        const vh = document.documentElement.clientHeight;
        const tipWidth = Math.min(600, vw - 24);
        let left = rect.left;

        if (left + tipWidth > vw - 12) left = vw - tipWidth - 12;
        if (left < 12) left = 12;

        const tipHeight = el.offsetHeight || Math.min(420, vh * 0.6);
        const topBelow = rect.bottom + 6;
        const topAbove = rect.top - tipHeight - 6;
        const belowFits = topBelow + tipHeight <= vh - 12;
        const aboveFits = topAbove >= 12;

        let top;
        if (belowFits) {
            top = topBelow;
        } else if (aboveFits) {
            top = topAbove;
        } else {
            // Neither fits cleanly — pick whichever clamped midpoint is nearest the link
            const topBelowClamped = vh - tipHeight - 12;
            const topAboveClamped = 12;
            const linkMid = rect.top + rect.height / 2;
            top = Math.abs(topBelowClamped + tipHeight / 2 - linkMid) <= Math.abs(topAboveClamped + tipHeight / 2 - linkMid)
                ? topBelowClamped : topAboveClamped;
        }

        el.style.left = left + 'px';
        el.style.top = top + 'px';
    }

    function showTooltip(anchor, frag, level) {
        // Hide any deeper tooltips before showing this level
        hideFrom(level + 1);

        const entry = getOrCreateLevel(level);
        entry.el.innerHTML = '';

        // Wrap in .rst-content so the theme's contextual CSS rules apply
        // (e.g. ".rst-content p.rubric { font-weight: 700 }")
        const wrapper = document.createElement('div');
        wrapper.className = 'rst-content';
        wrapper.appendChild(frag);
        entry.el.appendChild(wrapper);

        entry.el.style.top = '-9999px';
        entry.el.classList.add('visible');
        positionTooltip(entry.el, anchor);

        // Attach hover listeners to nested C/C++ API links for the next tooltip level.
        // Match on the anchor ID prefix: Sphinx C domain uses "#c." and C++ domain uses
        // "#_CPPv". This works for both Sphinx-prose xref links and Breathe function-
        // signature type links, and naturally excludes non-API internal links.
        entry.el.querySelectorAll(
            'a.reference.internal[href*="#c."], a.reference.internal[href*="#_CPPv"]'
        ).forEach(function (a) {
            attachHoverListeners(a, level + 1);
        });
    }

    function extractFragment(doc, anchorId, baseUrl) {
        const el = doc.getElementById(anchorId);
        if (!el) return null;
        const dl = el.closest('dl');
        if (!dl) return null;
        const clone = dl.cloneNode(true);
        clone.querySelectorAll('a.headerlink').forEach(function (a) { a.remove(); });
        // Resolve all hrefs to absolute URLs so recursive tooltips (level > 0) can
        // correctly fetch their targets regardless of the current page location.
        if (baseUrl) {
            clone.querySelectorAll('a[href]').forEach(function (a) {
                const href = a.getAttribute('href');
                if (href) {
                    try { a.setAttribute('href', new URL(href, baseUrl).href); } catch (e) {}
                }
            });
        }
        return clone;
    }

    async function loadTooltip(anchor, href, level) {
        const hashIdx = href.indexOf('#');
        const anchorId = hashIdx >= 0 ? href.slice(hashIdx + 1) : '';

        if (!anchorId) return;

        // Same-page anchor: extract directly from live document, no fetch needed
        if (hashIdx === 0) {
            const frag = extractFragment(document, anchorId, window.location.href);
            if (frag) showTooltip(anchor, frag, level);
            return;
        }

        // Normalise to an absolute URL so the cache key is stable and relative hrefs
        // inside fetched content (level > 0 tooltips) resolve against the right base.
        const pageUrl = new URL(href.slice(0, hashIdx), window.location.href).href;

        // If the target is the current page, use the live document directly.
        // Same-page level-0 hovers (hashIdx === 0) skip pageCache, so after
        // extractFragment rewrites "#TypeId" → "https://page.html#TypeId", a level-1
        // hover lands here with pageUrl = current page — not in cache and unfetchable
        // on file:// builds (browser blocks cross-origin fetch for local files).
        const currentPageUrl = window.location.href.split('#')[0];
        if (pageUrl === currentPageUrl) {
            const frag = extractFragment(document, anchorId, pageUrl);
            if (frag) showTooltip(anchor, frag, level);
            return;
        }

        try {
            if (!pageCache.has(pageUrl)) {
                const resp = await fetch(pageUrl);
                if (!resp.ok) return;
                const html = await resp.text();
                pageCache.set(pageUrl, new DOMParser().parseFromString(html, 'text/html'));
            }
            const frag = extractFragment(pageCache.get(pageUrl), anchorId, pageUrl);
            if (frag) showTooltip(anchor, frag, level);
        } catch (e) {
            // Silently ignore fetch errors (e.g. CORS, network)
        }
    }

    function attachHoverListeners(anchor, level) {
        let showTimer = null;
        anchor.removeAttribute('title');

        anchor.addEventListener('mouseenter', function () {
            // Keep all ancestor tooltips alive while hovering this link
            for (let i = 0; i < level; i++) {
                if (stack[i]) clearTimeout(stack[i].hideTimer);
            }
            clearTimeout(showTimer);
            const href = anchor.getAttribute('href') || '';
            showTimer = setTimeout(function () {
                loadTooltip(anchor, href, level);
            }, HOVER_DELAY_MS);
        });

        anchor.addEventListener('mouseleave', function () {
            clearTimeout(showTimer);
            scheduleHide(level);
        });
    }

    function init() {
        // Read computed styles from a live dt so the tooltip dt and frame look identical
        // to what is actually rendered in the API reference on this page.
        const refDt = document.querySelector(
            'dl[class]:not(.option-list):not(.field-list):not(.footnote):not(.citation):not(.glossary):not(.simple) > dt'
        );
        if (refDt) {
            const cs = getComputedStyle(refDt);
            const root = document.documentElement;
            root.style.setProperty('--hover-dt-bg',           cs.backgroundColor);
            root.style.setProperty('--hover-dt-color',        cs.color);
            root.style.setProperty('--hover-dt-border-top',   cs.borderTopWidth + ' ' + cs.borderTopStyle + ' ' + cs.borderTopColor);
            root.style.setProperty('--hover-dt-padding',      cs.padding);
            root.style.setProperty('--hover-dt-display',      cs.display);
            root.style.setProperty('--hover-api-accent',      cs.borderTopColor);
        }

        // Match C/C++ API links by anchor ID prefix: Sphinx C domain uses "#c." and
        // C++ domain uses "#_CPPv". This covers both Sphinx-prose xref links and Breathe
        // function-signature type/param links (which have no code.xref child element).
        document.querySelectorAll(
            'a.reference.internal[href*="#c."], a.reference.internal[href*="#_CPPv"]'
        ).forEach(function (a) {
            attachHoverListeners(a, 0);
        });

        // Hide all tooltips on scroll
        window.addEventListener('scroll', function () { hideFrom(0); }, {passive: true});
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
}());
