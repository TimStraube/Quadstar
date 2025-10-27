#!/bin/bash

echo "Welche Tests möchtest du ausführen?"
echo "1) Unit-Tests"
echo "2) Integrationstests"
echo "3) Beide"
read -p "Bitte wähle (1/2/3): " choice

if [[ "$choice" == "1" ]]; then
    echo "\nRunning unit tests..."
    for testfile in test.py visualizer.py; do
        echo "----------------------------------------"
        echo "Running test/unit/$testfile"
        (cd test/unit && python3 "$testfile")
    done
elif [[ "$choice" == "2" ]]; then
    echo "Running integration tests..."
    for testfile in test.py visualizer_dependency.py visualizer.py; do
        echo "----------------------------------------"
        echo "Running test/integration/$testfile"
        (cd test/integration && python3 "$testfile")
    done
elif [[ "$choice" == "3" ]]; then
    echo "\nRunning unit tests..."
    for testfile in test.py visualizer.py; do
        echo "----------------------------------------"
        echo "Running test/unit/$testfile"
        (cd test/unit && python3 "$testfile")
    done
    echo "Running integration tests..."
    for testfile in test.py visualizer_dependency.py visualizer.py; do
        echo "----------------------------------------"
        echo "Running test/integration/$testfile"
        (cd test/integration && python3 "$testfile")
    done
else
    echo "Ungültige Auswahl."
    exit 1
fi

