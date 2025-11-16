#!/bin/bash

# Name des Environments
ENV_NAME="quadstar"
PYTHON_VERSION="3.13.1"

# Prüfen, ob pyenv installiert ist
if ! command -v pyenv &> /dev/null; then
    echo "pyenv ist nicht installiert. Bitte installiere pyenv zuerst."
    exit 1
fi

# Prüfen, ob das Environment bereits existiert
if ! pyenv virtualenvs --bare | grep -q "^${ENV_NAME}\$"; then
    # Python-Version installieren, falls nicht vorhanden
    if ! pyenv versions --bare | grep -q "^${PYTHON_VERSION}\$"; then
        pyenv install ${PYTHON_VERSION}
    fi
    # Environment erstellen
    pyenv virtualenv ${PYTHON_VERSION} ${ENV_NAME}
    echo "pyenv-Environment '${ENV_NAME}' wurde erstellt."
else
    echo "pyenv-Environment '${ENV_NAME}' existiert bereits."
fi

# Environment aktivieren
eval "$(pyenv init -)"
pyenv activate ${ENV_NAME}

# Poetry installieren, falls nicht vorhanden
if ! command -v poetry &> /dev/null; then
    pip install poetry
fi

# Abhängigkeiten installieren
poetry install

echo "Alle Bibliotheken wurden installiert."