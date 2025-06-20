# --------------------------------------------------
# File: config_manager.py
# --------------------------------------------------
import json

CONFIG_PATH = "assets/config.json"


def load_config() -> dict:
    try:
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def save_config(config: dict):
    try:
        with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
            json.dump(config, f, indent=4)
    except Exception as e:
        raise IOError(f"Erro ao salvar configuração: {e}")
