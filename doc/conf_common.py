from esp_docs.conf_docs import *  # noqa: F403,F401
from esp_docs.conf_docs import setup as _esp_docs_setup

extensions += ['sphinx_copybutton',
               # Needed as a trigger for running doxygen
               'esp_docs.esp_extensions.dummy_build_system',
               'esp_docs.esp_extensions.run_doxygen',
               'myst_parser',
               'sphinxcontrib.mermaid',
               ]

mermaid_output_format = 'raw'
mermaid_d3_zoom = True
mermaid_dark_theme = 'neutral'
mermaid_light_theme = 'neutral'

exclude_patterns = ['build', '_build', 'detail']

# link roles config
github_repo = 'esp-cpp/espp'

# --- theme -------------------------------------------------------------------
# pydata-sphinx-theme instead of esp-docs' sphinx_idf_theme: a top bar, the
# section navigation on the left, the page outline on the right, light and dark
# modes, a version switcher driven by a JSON file we host (wired up by the
# versioned-publishing workflow), and no chip selector or ESP-IDF version list.
# esp-docs still runs Doxygen and provides the API directives; only the look
# changes.
html_theme = 'pydata_sphinx_theme'
# esp-docs loads its theme as an extension too, and that extension's page hook
# demands the chip and language selectors' config; neither is wanted here
extensions = [ext for ext in extensions if ext != 'sphinx_idf_theme']
html_static_path = ['_static']
html_css_files = ['espp.css']
html_title = 'espp'
html_favicon = '_static/favicon.png'

# the "edit this page" button and the GitHub link
html_context['github_user'] = 'esp-cpp'
html_context['github_repo'] = 'espp'
html_context['github_version'] = 'main'
html_context['doc_path'] = 'doc/en'

html_theme_options = {
    'logo': {
        'text': 'espp',
        'image_light': '_static/espp-logo.png',
        'image_dark': '_static/espp-logo.png',
    },
    'github_url': 'https://github.com/esp-cpp/espp',
    'icon_links': [
        {
            'name': 'Discord',
            'url': 'https://discord.gg/dvcQw37xAY',
            'icon': 'fa-brands fa-discord',
        },
    ],
    'use_edit_page_button': True,
    'navbar_align': 'left',
    'header_links_before_dropdown': 5,
    'show_toc_level': 2,
    'navigation_with_keys': False,
    'show_prev_next': True,
    'secondary_sidebar_items': ['page-toc', 'edit-this-page'],
    'footer_start': ['copyright'],
    'footer_end': ['sphinx-version', 'theme-version'],
    'pygments_light_style': 'default',
    'pygments_dark_style': 'monokai',
    # filled in by the publishing workflow (versioned docs): the JSON lists
    # "latest" (main) and one entry per release/v* branch
    # 'switcher': {'json_url': 'https://esp-cpp.github.io/espp/versions.json',
    #              'version_match': version_match},
}


def _espp_html(app, config):
    # esp-docs' own config-inited hook (setup_html) points html_logo at the
    # Espressif logo; this runs after it (higher priority number) and gives the
    # theme ours instead (the theme takes the logo from html_theme_options).
    config.html_logo = None


def setup(app):
    # esp-docs' setup registers its Doxygen / API extensions and hooks; keep all
    # of that, then add the hook above behind them.
    ret = _esp_docs_setup(app)
    # Config values the dropped theme extension used to register, which
    # esp-docs' LaTeX and html-zip extensions still read (project_slug,
    # html_zip ...). A conf.py setup runs after every extension's, so these
    # exist before the config-inited handlers that read them.
    for name, default in [('project_slug', 'espp'), ('versions_url', ''), ('project_homepage', ''),
                          ('languages', ['en']), ('download_url', ''),
                          ('latest_branch_name', 'main'), ('html_zip', '')]:
        if name not in app.config:
            app.add_config_value(name, default, 'html')
    if 'idf_targets' not in app.config:
        app.add_config_value('idf_targets', None, 'env')
    app.connect('config-inited', _espp_html, priority=900)
    return ret
