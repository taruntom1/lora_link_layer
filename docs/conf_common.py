# Common Sphinx configuration shared across languages.

extensions = [
    "breathe",
]

github_repo = "lora_link_layer"
github_user = "taruntom1"

project_slug = "lora-link-layer"
project_homepage = "https://github.com/taruntom1/lora_link_layer"

idf_targets = ["esp32s3"]
idf_target_title_dict = {
    "esp32s3": "ESP32-S3",
}

languages = ["en"]

pdf_file = ""

html_context = {
    "github_user": github_user,
    "github_repo": github_repo,
}

breathe_projects = {
    "lora_radio": "../doxygen/xml",
}
breathe_default_project = "lora_radio"

nitpicky = False
