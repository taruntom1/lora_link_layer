# Common Sphinx configuration shared across languages.

extensions = [
    "breathe",
]

github_repo = "lora_link_layer"
github_user = "taruntom1"

project_slug = "lora-link-layer"
project_homepage = "https://github.com/taruntom1/lora_link_layer"

idf_targets = ["esp32s3"]

languages = ["en"]

html_context = {
    "github_user": github_user,
    "github_repo": github_repo,
}

breathe_projects = {
    "lora_radio": "../doxygen/xml",
}
breathe_default_project = "lora_radio"

nitpicky = False
