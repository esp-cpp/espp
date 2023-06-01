from esp_docs.conf_docs import *  # noqa: F403,F401

extensions += ['sphinx_copybutton',
               # Needed as a trigger for running doxygen
               'esp_docs.esp_extensions.dummy_build_system',
               'esp_docs.esp_extensions.run_doxygen',
               ]

exclude_paterns = ['build', '_build']

# link roles config
github_repo = 'esp-cpp/espp'

# context used by sphinx_idf_theme
html_context['github_user'] = 'esp-cpp'
html_context['github_repo'] = 'espp'

# Extra options required by sphinx_idf_theme
project_slug = 'esp-cpp'
versions_url = 'https://dl.espressif.com/dl/esp-idf/idf_versions.js'

idf_targets = ['esp32', 'esp32s2', 'esp32c3']
languages = ['en']
