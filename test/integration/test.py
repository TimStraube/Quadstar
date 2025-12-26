import json
import os
import itertools
from datetime import datetime

class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def get_setup_variants(setup):
    if not setup:
        return [{}]
    keys = list(setup.keys())
    options = [setup[k]["options"] for k in keys]
    return [dict(zip(keys, combination)) for combination in itertools.product(*options)]

def get_test_results(test_definitions):
    results = {}
    for test_number, test in test_definitions.items():
        if not isinstance(test, dict):
            continue  # Nur echte Testobjekte verarbeiten
        print(f"\n{Colors.BOLD}{'='*80}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.BLUE}Test #{test_number}: {test.get('name', '')}{Colors.ENDC}")
        print(f"{Colors.BOLD}{'='*80}{Colors.ENDC}")
        print(f"\n{Colors.YELLOW}Beschreibung:{Colors.ENDC}")
        print(f"{Colors.CYAN}{test.get('description', '')}{Colors.ENDC}")

        executions = test.get('executions', 1)
        targets = test.get('targets', [])
        setup = test.get('setup', {})

        setup_variants = get_setup_variants(setup)
        print(f"\n{Colors.YELLOW}Setup-Varianten: {len(setup_variants)}{Colors.ENDC}")

        variant_results = []
        for variant_idx, variant in enumerate(setup_variants, 1):
            print(f"\n{Colors.YELLOW}Setup-Variante {variant_idx}/{len(setup_variants)}:{Colors.ENDC}")
            if variant:
                for key, value in variant.items():
                    print(f"  {key}: {Colors.YELLOW}{value}{Colors.ENDC}")
            else:
                print("  Standard")
            executions_results = []
            for i in range(1, executions + 1):
                print(f"\n{Colors.GREEN}Durchführung {i} von {executions} für Variante {variant_idx}{Colors.ENDC}")
                target_results = {}
                for target in targets:
                    answer = input(f"{Colors.BOLD}Target '{target}': Enter für bestanden, 'N' für nicht getestet, sonst Kommentar für fehlgeschlagen: {Colors.ENDC}").strip()
                    if not answer:
                        status = "bestanden"
                        status_emoji = "🟢"
                        comment = ""
                        print(f"{Colors.GREEN}Status: {status_emoji} {status}{Colors.ENDC}")
                    elif answer.lower() == 'n':
                        status = "nicht getestet"
                        status_emoji = "⚪"
                        comment = ""
                        print(f"{Colors.YELLOW}Status: {status_emoji} {status}{Colors.ENDC}")
                    else:
                        status = "fehlgeschlagen"
                        status_emoji = "🔴"
                        comment = answer
                        print(f"{Colors.RED}Status: {status_emoji} {status} - {comment}{Colors.ENDC}")
                    target_results[target] = {
                        "status": status,
                        "status_emoji": status_emoji,
                        "comment": comment
                    }
                executions_results.append({
                    "execution_number": i,
                    "targets": target_results,
                    "setup": variant
                })
                if i < executions:
                    print(f"{Colors.BOLD}{'-'*40}{Colors.ENDC}")
            variant_results.append({
                "setup": variant,
                "executions": executions_results
            })
        results[test_number] = {
            "name": test.get('name', ''),
            "variants": variant_results
        }
    return results

def select_test_file():
    files = [f for f in os.listdir('.') if f.endswith('.json')]
    if not files:
        print(f"{Colors.RED}Keine Testdateien (.json) im aktuellen Ordner gefunden!{Colors.ENDC}")
        exit(1)
    print(f"\n{Colors.BOLD}Verfügbare Testdateien:{Colors.ENDC}")
    for idx, fname in enumerate(files, 1):
        print(f"{idx}: {fname}")
    while True:
        choice = input(f"{Colors.BOLD}Bitte wähle eine Datei (Nummer): {Colors.ENDC}").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(files):
            return files[int(choice)-1]
        print(f"{Colors.RED}Ungültige Auswahl!{Colors.ENDC}")

def load_test_definitions(filename):
    try:
        with open(filename, "r", encoding="utf-8") as f:
            test_definitions = json.load(f)
        print(f"{Colors.GREEN}Testdefinitionen erfolgreich geladen aus: {filename}{Colors.ENDC}")
        return test_definitions
    except FileNotFoundError:
        print(f"{Colors.RED}Fehler: Datei '{filename}' nicht gefunden!{Colors.ENDC}")
        exit(1)
    except json.JSONDecodeError as e:
        print(f"{Colors.RED}Fehler beim Parsen der JSON-Datei '{filename}': {e}{Colors.ENDC}")
        exit(1)
    except Exception as e:
        print(f"{Colors.RED}Unerwarteter Fehler beim Laden der Testdefinitionen: {e}{Colors.ENDC}")
        exit(1)

def save_results_to_json(results, app_info):
    results_dir = "results"
    os.makedirs(results_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{results_dir}/test_results_{app_info['project']}_v{app_info['version']}_{timestamp}.json"
    results_with_info = {
        "app_info": app_info,
        "tests": results
    }
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(results_with_info, f, indent=4, ensure_ascii=False)
    print(f"\nErgebnisse gespeichert in {filename}")

def main():
    print(f"\n{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}                    THETAFLY INTEGRATION TEST                        {Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")
    project = input(f"\n{Colors.BOLD}Bitte gib den Projektnamen ein: {Colors.ENDC}") or "FREAK"
    version = input(f"{Colors.BOLD}Bitte gib die Version ein [optional]: {Colors.ENDC}").strip()
    if not version:
        version = "0.0.0"
    app_info = {
        "project": project,
            "version": version,
        "test_date": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    test_file = select_test_file()
    test_definitions = load_test_definitions(test_file)
    results = get_test_results(test_definitions)
    save_results_to_json(results, app_info)
    print(f"\n{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}                    TEST ABGESCHLOSSEN                        {Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")

if __name__ == "__main__":
    main()