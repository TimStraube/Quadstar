import json
import os
from datetime import datetime

# ANSI-Farbcodes für Konsolenausgabe
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

def get_test_results(test_definitions):
    """Führt den Benutzer durch die Tests und erfasst die Ergebnisse."""
    results = {}
    
    for test_number, test in test_definitions.items():
        print(f"\n{Colors.BOLD}{'='*80}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.BLUE}Test #{test_number}: {test['name']}{Colors.ENDC}")
        print(f"{Colors.BOLD}{'='*80}{Colors.ENDC}")
        
        print(f"\n{Colors.YELLOW}Beschreibung:{Colors.ENDC}")
        print(f"{Colors.CYAN}{test['description']}{Colors.ENDC}")
        
        print(f"\n{Colors.YELLOW}Dieser Test muss {Colors.BOLD}{test['executions']}{Colors.ENDC}{Colors.YELLOW} mal durchgeführt werden{Colors.ENDC}")
        print(f"\n{Colors.BOLD}{'-'*80}{Colors.ENDC}")
        
        test_executions = []
        for i in range(1, test['executions'] + 1):
            print(f"\n{Colors.GREEN}Durchführung {i} von {test['executions']} für Test #{test_number}{Colors.ENDC}")
            answer = input(f"{Colors.BOLD}Enter für bestanden, 'N' für nicht getestet, sonst Kommentar für fehlgeschlagen: {Colors.ENDC}").strip()
            
            if not answer:
                # Benutzer hat einfach Enter gedrückt -> Test bestanden
                status = "bestanden"
                status_emoji = "🟢"
                comment = ""
                print(f"{Colors.GREEN}Status: {status_emoji} {status}{Colors.ENDC}")
            elif answer.lower() == 'n':
                # Benutzer hat 'N' eingegeben -> Test nicht getestet
                status = "nicht getestet"
                status_emoji = "⚪"
                comment = ""
                print(f"{Colors.YELLOW}Status: {status_emoji} {status}{Colors.ENDC}")
            else:
                # Benutzer hat einen Kommentar eingegeben -> Test fehlgeschlagen
                status = "fehlgeschlagen"
                status_emoji = "🔴"
                comment = answer
                print(f"{Colors.RED}Status: {status_emoji} {status} - {comment}{Colors.ENDC}")
                
            test_executions.append({
                "execution_number": i,
                "status": status,
                "status_emoji": status_emoji,
                "comment": comment
            })
            
            # Trennlinie zwischen den Durchführungen
            if i < test['executions']:
                print(f"{Colors.BOLD}{'-'*40}{Colors.ENDC}")
        
        # Gesamtergebnis des Tests bestimmen
        overall_status = "bestanden"
        overall_status_emoji = "🟢"
        for execution in test_executions:
            if execution["status"] == "fehlgeschlagen":
                overall_status = "fehlgeschlagen"
                overall_status_emoji = "🔴"
                break
        
        # Ausgabe des Gesamtergebnisses
        status_color = Colors.GREEN if overall_status == "bestanden" else Colors.RED
        print(f"\n{status_color}Gesamtergebnis: {overall_status_emoji} {overall_status.upper()}{Colors.ENDC}")
        
        results[test_number] = {
            "name": test['name'],
            "overall_status": overall_status,
            "overall_status_emoji": overall_status_emoji,
            "executions": test_executions
        }
    
    return results

def load_test_definitions(filename="test_esc.json"):
    """Lädt die Testdefinitionen aus einer JSON-Datei."""
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, filename)
        
        with open(file_path, "r", encoding="utf-8") as f:
            test_definitions = json.load(f)
        
        print(f"{Colors.GREEN}Testdefinitionen erfolgreich geladen aus: {filename}{Colors.ENDC}")
        return test_definitions
        
    except FileNotFoundError:
        print(f"{Colors.RED}Fehler: Datei '{filename}' nicht gefunden im Verzeichnis: {script_dir}{Colors.ENDC}")
        print(f"{Colors.YELLOW}Bitte stelle sicher, dass die Datei '{filename}' im gleichen Verzeichnis wie das Skript existiert.{Colors.ENDC}")
        exit(1)
    except json.JSONDecodeError as e:
        print(f"{Colors.RED}Fehler beim Parsen der JSON-Datei '{filename}': {e}{Colors.ENDC}")
        exit(1)
    except Exception as e:
        print(f"{Colors.RED}Unerwarteter Fehler beim Laden der Testdefinitionen: {e}{Colors.ENDC}")
        exit(1)

def save_results_to_json(results, app_info):
    """Speichert die Testergebnisse in einer JSON-Datei mit dem Versionsnamen."""
    # Erstelle 'results' Ordner, falls er nicht existiert
    results_dir = "results"
    os.makedirs(results_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{results_dir}/test_results_v{app_info['app_version']}_{timestamp}.json"
    
    # Füge App-Informationen zum Ergebnis hinzu
    results_with_info = {
        "app_info": app_info,
        "tests": results
    }
    
    with open(filename, "w", encoding="utf-8") as f:
        json.dump(results_with_info, f, indent=4, ensure_ascii=False)
    print(f"\nErgebnisse gespeichert in {filename}")

def main():
    """Hauptfunktion des Skripts."""
    
    print(f"\n{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}                    QUADSTAR UNIT TEST                        {Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")
    
    # App-Informationen erfragen
    app_version = input(f"\n{Colors.BOLD}Bitte gib die App-Version ein (z.B. 1.0.0) [optional]: {Colors.ENDC}") or "unbekannt"
    android_build = input(f"{Colors.BOLD}Android Build-Nummer (Google Play/Android Studio) [optional]: {Colors.ENDC}") or "unbekannt"
    ios_build = input(f"{Colors.BOLD}iOS Build-Nummer (Apple App Store) [optional]: {Colors.ENDC}") or "unbekannt"
    
    app_info = {
        "app_version": app_version,
        "android_build": android_build,
        "ios_build": ios_build,
        "test_date": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    
    # Testdefinitionen aus JSON-Datei laden
    test_definitions = load_test_definitions()

    results = get_test_results(test_definitions)
    save_results_to_json(results, app_info)
    
    print(f"\n{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}                    TEST ABGESCHLOSSEN                        {Colors.ENDC}")
    print(f"{Colors.BOLD}{Colors.HEADER}{'='*80}{Colors.ENDC}")

if __name__ == "__main__":
    main()