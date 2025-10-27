import json
import os
import glob
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import argparse
import matplotlib.font_manager as fm
from matplotlib.patches import Wedge
from collections import Counter

def parse_timestamp(filename):
    """Extrahiert Zeitstempel und Version aus dem Dateinamen."""
    # Format: test_results_v1.0.0_20231206_120000.json
    parts = os.path.basename(filename).replace('.json', '').split('_')
    version = parts[2][1:]  # Entferne 'v' aus der Version
    timestamp = '_'.join(parts[3:])
    return version, datetime.strptime(timestamp, '%Y%m%d_%H%M%S')

def select_results_file():
    """Lässt den Benutzer eine Ergebnisdatei aus dem results-Ordner auswählen."""
    results_dir = "results"
    
    if not os.path.exists(results_dir):
        print(f"Der Ordner '{results_dir}' existiert nicht.")
        return None
    
    json_files = glob.glob(f"{results_dir}/test_results_*.json")
    if not json_files:
        print(f"Keine Testergebnisse im Ordner '{results_dir}' gefunden.")
        return None
    
    # Sortiere nach Zeitstempel (neueste zuerst)
    json_files.sort(key=lambda x: parse_timestamp(x)[1], reverse=True)
    
    print("\nVerfügbare Testergebnisse:")
    for i, file in enumerate(json_files):
        version, timestamp = parse_timestamp(file)
        
        # Lade die Datei, um den Status anzuzeigen
        try:
            with open(file, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            # Bestimme den Gesamtstatus aller Tests
            # Anpassen für die neue Struktur
            tests_data = data.get("tests", data)  # Fallback für alte Dateien
            
            all_passed = True
            for test_data in tests_data.values():
                if test_data.get("overall_status", "") == "fehlgeschlagen":
                    all_passed = False
                    break
            
            status_emoji = "🟢" if all_passed else "🔴"
        except Exception:
            status_emoji = "⚪"  # Neutrales Emoji bei Fehler
        
        print(f"{i+1}. {status_emoji} App-Version {version} vom {timestamp.strftime('%d.%m.%Y %H:%M:%S')}")
    
    while True:
        try:
            choice = input("\nWähle eine Datei (Nummer) oder drücke Enter für die neueste: ")
            if not choice:
                return json_files[0]
            
            choice = int(choice)
            if 1 <= choice <= len(json_files):
                return json_files[choice-1]
            else:
                print(f"Bitte wähle eine Zahl zwischen 1 und {len(json_files)}.")
        except ValueError:
            print("Bitte gib eine gültige Zahl ein.")

def load_results(filename=None):
    """Lädt die Testergebnisse aus einer JSON-Datei."""
    if filename is None:
        filename = select_results_file()
        if not filename:
            return None, None, None
    
    try:
        with open(filename, "r", encoding="utf-8") as f:
            data = json.load(f)
        
        # Unterstütze sowohl das neue als auch das alte Format
        if "app_info" in data and "tests" in data:
            # Neues Format
            app_info = data["app_info"]
            results = data["tests"]
            version = app_info["app_version"]
        else:
            # Altes Format
            app_info = {"app_version": parse_timestamp(filename)[0]}
            results = data
            version = app_info["app_version"]
            
        return results, version, app_info
    except Exception as e:
        print(f"Fehler beim Laden der Datei {filename}: {e}")
        return None, None, None

def visualize_results(results, version, app_info=None):
    """Visualisiert die Testergebnisse mit mehreren coolen Grafiken."""
    if not results:
        return
    
    # Erstelle einen Ordner für die Diagramme, falls er nicht existiert
    output_dir = "results/visualizations"
    os.makedirs(output_dir, exist_ok=True)
    
    # Sammle alle Statistiken
    all_stats = collect_test_statistics(results)
    
    # 1. Detaillierte Balkendiagramme für jeden Test
    create_detailed_bar_charts(results, version, app_info, output_dir)
    
    # 2. Gesamtübersicht als Heatmap
    create_overview_heatmap(results, version, app_info, output_dir, all_stats)
    
    # 3. Pie Charts für Status-Verteilung
    create_status_pie_charts(results, version, app_info, output_dir, all_stats)
    
    # 4. Radar Chart für Test-Performance
    create_radar_chart(results, version, app_info, output_dir, all_stats)
    
    # 5. Timeline/Trend Chart (wenn mehrere Durchführungen)
    create_execution_timeline(results, version, app_info, output_dir)
    
    # 6. Test Coverage Matrix
    create_test_coverage_matrix(results, version, app_info, output_dir, all_stats)
    
    print(f"\n🎨 Visualisierungen wurden im Ordner '{output_dir}' gespeichert:")
    print(f"   📊 Detaillierte Ergebnisse: detail_results_v{version}.png")
    print(f"   🔥 Übersichts-Heatmap: overview_heatmap_v{version}.png")
    print(f"   🥧 Status-Verteilung: status_distribution_v{version}.png")
    print(f"   🕸️  Performance-Radar: performance_radar_v{version}.png")
    print(f"   📈 Ausführungs-Timeline: execution_timeline_v{version}.png")
    print(f"   🎯 Test-Coverage-Matrix: test_coverage_v{version}.png")

def collect_test_statistics(results):
    """Sammelt alle relevanten Statistiken aus den Testergebnissen."""
    stats = {
        'total_tests': len(results),
        'total_executions': 0,
        'passed': 0,
        'failed': 0,
        'not_tested': 0,
        'success_rates': [],
        'test_names': [],
        'execution_counts': [],
        'status_distribution': Counter()
    }
    
    for test_id, test_data in results.items():
        executions = test_data["executions"]
        stats['total_executions'] += len(executions)
        stats['test_names'].append(test_data["name"])
        stats['execution_counts'].append(len(executions))
        
        # Zähle Statuses
        passed = sum(1 for e in executions if e["status"] == "bestanden")
        failed = sum(1 for e in executions if e["status"] == "fehlgeschlagen")
        not_tested = sum(1 for e in executions if e["status"] == "nicht getestet")
        
        stats['passed'] += passed
        stats['failed'] += failed
        stats['not_tested'] += not_tested
        
        # Status-Verteilung für Counter
        for execution in executions:
            stats['status_distribution'][execution["status"]] += 1
        
        # Success Rate berechnen (nur getestete Durchführungen)
        tested_executions = passed + failed
        if tested_executions > 0:
            success_rate = passed / tested_executions * 100
        else:
            success_rate = 0  # Wenn nichts getestet wurde
        stats['success_rates'].append(success_rate)
    
    return stats

def create_detailed_bar_charts(results, version, app_info, output_dir):
    """Erstellt detaillierte Balkendiagramme für jeden Test."""
    num_tests = len(results)
    fig, axes = plt.subplots(num_tests, 1, figsize=(12, 6*num_tests))
    if num_tests == 1:
        axes = [axes]
    
    colors = ['#2ECC71', '#E74C3C', '#95A5A6']  # Grün, Rot, Grau
    
    for idx, (test_id, test_data) in enumerate(results.items()):
        test_name = test_data["name"]
        executions = test_data["executions"]
        
        # Zähle alle drei Status-Typen
        passed = sum(1 for e in executions if e["status"] == "bestanden")
        failed = sum(1 for e in executions if e["status"] == "fehlgeschlagen")
        not_tested = sum(1 for e in executions if e["status"] == "nicht getestet")
        
        ax = axes[idx]
        categories = ["✅ Bestanden", "❌ Fehlgeschlagen", "⚪ Nicht getestet"]
        counts = [passed, failed, not_tested]
        
        bars = ax.bar(categories, counts, color=colors)
        ax.set_title(f"Test #{test_id}: {test_name}", fontsize=14, fontweight='bold')
        ax.set_ylabel("Anzahl der Durchführungen")
        
        # Füge Beschriftungen zu den Balken hinzu
        for bar, count in zip(bars, counts):
            if count > 0:
                ax.annotate(f'{count}',
                           xy=(bar.get_x() + bar.get_width() / 2, count),
                           xytext=(0, 3),
                           textcoords="offset points",
                           ha='center', va='bottom', fontweight='bold')
        
        # Grid für bessere Lesbarkeit
        ax.grid(axis='y', alpha=0.3)
        ax.set_ylim(0, max(counts) * 1.1 if max(counts) > 0 else 1)
    
    create_title_with_info(fig, f"Detaillierte Testergebnisse - App-Version {version}", app_info)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/detail_results_v{version}.png", dpi=300, bbox_inches='tight')
    plt.close()

def create_overview_heatmap(results, version, app_info, output_dir, stats):
    """Erstellt eine Heatmap-Übersicht aller Tests."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Left: Success rates heatmap
    test_names = [f"#{i+1}: {name[:20]}..." if len(name) > 20 else f"#{i+1}: {name}" 
                  for i, name in enumerate(stats['test_names'])]
    success_rates = np.array(stats['success_rates']).reshape(-1, 1)
    
    im1 = ax1.imshow(success_rates, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)
    ax1.set_title("Erfolgsraten der Tests", fontsize=14, fontweight='bold')
    ax1.set_yticks(range(len(test_names)))
    ax1.set_yticklabels(test_names)
    ax1.set_xticks([])
    
    # Füge Prozent-Werte hinzu
    for i, rate in enumerate(stats['success_rates']):
        color = 'white' if rate < 50 else 'black'
        ax1.text(0, i, f"{rate:.1f}%", ha='center', va='center', 
                color=color, fontweight='bold')
    
    # Colorbar für Heatmap
    cbar1 = plt.colorbar(im1, ax=ax1)
    cbar1.set_label('Erfolgsrate (%)', rotation=270, labelpad=20)
    
    # Right: Execution count heatmap
    execution_counts = np.array(stats['execution_counts']).reshape(-1, 1)
    max_executions = max(stats['execution_counts']) if stats['execution_counts'] else 1
    
    im2 = ax2.imshow(execution_counts, cmap='Blues', aspect='auto', vmin=0, vmax=max_executions)
    ax2.set_title("Anzahl Durchführungen", fontsize=14, fontweight='bold')
    ax2.set_yticks(range(len(test_names)))
    ax2.set_yticklabels(test_names)
    ax2.set_xticks([])
    
    # Füge Anzahl-Werte hinzu
    for i, count in enumerate(stats['execution_counts']):
        ax2.text(0, i, str(count), ha='center', va='center', 
                color='white' if count > max_executions/2 else 'black', fontweight='bold')
    
    # Colorbar für Ausführungen
    cbar2 = plt.colorbar(im2, ax=ax2)
    cbar2.set_label('Anzahl Durchführungen', rotation=270, labelpad=20)
    
    create_title_with_info(fig, f"Test-Übersicht Heatmap - App-Version {version}", app_info)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/overview_heatmap_v{version}.png", dpi=300, bbox_inches='tight')
    plt.close()

def create_status_pie_charts(results, version, app_info, output_dir, stats):
    """Erstellt Pie Charts für die Status-Verteilung."""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 12))
    
    # 1. Gesamtverteilung aller Ausführungen
    labels = ['Bestanden', 'Fehlgeschlagen', 'Nicht getestet']
    sizes = [stats['passed'], stats['failed'], stats['not_tested']]
    colors = ['#2ECC71', '#E74C3C', '#95A5A6']
    explode = (0.05, 0.05, 0.05)
    
    wedges, texts, autotexts = ax1.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%',
                                       explode=explode, shadow=True, startangle=90)
    ax1.set_title('Gesamtverteilung aller Durchführungen', fontweight='bold')
    
    # 2. Test-Erfolgsrate (Tests, die 100% bestanden haben vs. andere)
    perfect_tests = sum(1 for rate in stats['success_rates'] if rate == 100)
    imperfect_tests = len(stats['success_rates']) - perfect_tests
    
    ax2.pie([perfect_tests, imperfect_tests], 
            labels=['100% erfolgreich', 'Mit Fehlern/Auslassungen'],
            colors=['#27AE60', '#F39C12'], autopct='%1.1f%%', startangle=90)
    ax2.set_title('Tests nach Perfektion', fontweight='bold')
    
    # 3. Donut Chart für detaillierte Status-Verteilung
    status_data = stats['status_distribution']
    if status_data:
        labels = list(status_data.keys())
        sizes = list(status_data.values())
        
        wedges, texts, autotexts = ax3.pie(sizes, labels=labels, colors=colors[:len(labels)], 
                                          autopct='%1.1f%%', startangle=90, 
                                          wedgeprops=dict(width=0.5))
        ax3.set_title('Status-Verteilung (Donut)', fontweight='bold')
    
    # 4. Test Coverage (getestet vs. nicht getestet)
    tested_executions = stats['passed'] + stats['failed']
    not_tested_executions = stats['not_tested']
    
    if tested_executions + not_tested_executions > 0:
        ax4.pie([tested_executions, not_tested_executions],
                labels=['Getestet', 'Nicht getestet'],
                colors=['#3498DB', '#95A5A6'], autopct='%1.1f%%', startangle=90)
        ax4.set_title('Test-Abdeckung', fontweight='bold')
    
    create_title_with_info(fig, f"Status-Verteilungen - App-Version {version}", app_info)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/status_distribution_v{version}.png", dpi=300, bbox_inches='tight')
    plt.close()

def create_radar_chart(results, version, app_info, output_dir, stats):
    """Erstellt ein Radar Chart für Test-Performance."""
    if len(results) < 3:
        return  # Radar Charts brauchen mindestens 3 Datenpunkte
    
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
    
    # Vorbereitung der Daten
    test_names = [name[:15] + "..." if len(name) > 15 else name for name in stats['test_names']]
    success_rates = stats['success_rates']
    
    # Winkel für jeden Test
    angles = np.linspace(0, 2 * np.pi, len(test_names), endpoint=False).tolist()
    success_rates += [success_rates[0]]  # Schließe den Kreis
    angles += [angles[0]]
    
    # Zeichne das Radar Chart
    ax.plot(angles, success_rates, 'o-', linewidth=2, color='#3498DB')
    ax.fill(angles, success_rates, alpha=0.25, color='#3498DB')
    
    # Setze Labels
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(test_names)
    ax.set_ylim(0, 100)
    ax.set_yticks([20, 40, 60, 80, 100])
    ax.set_yticklabels(['20%', '40%', '60%', '80%', '100%'])
    ax.grid(True)
    
    # Titel
    ax.set_title(f"Test-Performance Radar\nApp-Version {version}", 
                 pad=20, fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/performance_radar_v{version}.png", dpi=300, bbox_inches='tight')
    plt.close()

def create_execution_timeline(results, version, app_info, output_dir):
    """Erstellt eine Timeline der Test-Ausführungen."""
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Sammle Daten für Timeline
    y_pos = 0
    colors = {'bestanden': '#2ECC71', 'fehlgeschlagen': '#E74C3C', 'nicht getestet': '#95A5A6'}
    
    for test_id, test_data in results.items():
        test_name = test_data["name"]
        executions = test_data["executions"]
        
        # Zeichne jeden Execution-Status
        for i, execution in enumerate(executions):
            status = execution["status"]
            color = colors[status]
            
            # Rechteck für jede Ausführung
            rect = plt.Rectangle((i, y_pos), 0.8, 0.8, facecolor=color, alpha=0.7, edgecolor='black')
            ax.add_patch(rect)
            
            # Nummer in das Rechteck
            ax.text(i + 0.4, y_pos + 0.4, str(i+1), ha='center', va='center', 
                   fontweight='bold', color='white' if status != 'nicht getestet' else 'black')
        
        # Test-Label
        ax.text(-1, y_pos + 0.4, f"Test #{test_id}: {test_name[:30]}...", 
               ha='right', va='center', fontweight='bold')
        
        y_pos += 1
    
    # Achsen-Einstellungen
    ax.set_xlim(-1, max(len(test_data["executions"]) for test_data in results.values()))
    ax.set_ylim(-0.5, len(results) - 0.5)
    ax.set_xlabel('Ausführung Nr.', fontweight='bold')
    ax.set_ylabel('Tests', fontweight='bold')
    ax.set_title(f'Test-Ausführungs-Timeline - App-Version {version}', 
                fontsize=14, fontweight='bold')
    
    # Legende
    legend_elements = [plt.Rectangle((0,0),1,1, facecolor=color, label=status.title()) 
                      for status, color in colors.items()]
    ax.legend(handles=legend_elements, loc='upper right')
    
    ax.grid(axis='x', alpha=0.3)
    plt.tight_layout()
    plt.savefig(f"{output_dir}/execution_timeline_v{version}.png", dpi=300, bbox_inches='tight')
    plt.close()

def create_test_coverage_matrix(results, version, app_info, output_dir, stats):
    """Erstellt eine Test-Coverage-Matrix."""
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Matrix-Daten vorbereiten
    test_names = stats['test_names']
    max_executions = max(stats['execution_counts']) if stats['execution_counts'] else 1
    
    # Matrix erstellen
    matrix = np.zeros((len(test_names), max_executions))
    
    for i, (test_id, test_data) in enumerate(results.items()):
        executions = test_data["executions"]
        for j, execution in enumerate(executions):
            if execution["status"] == "bestanden":
                matrix[i][j] = 1
            elif execution["status"] == "fehlgeschlagen":
                matrix[i][j] = -1
            else:  # nicht getestet
                matrix[i][j] = 0
    
    # Custom Colormap
    colors = ['#E74C3C', '#95A5A6', '#2ECC71']  # Rot, Grau, Grün
    from matplotlib.colors import ListedColormap
    custom_cmap = ListedColormap(colors)
    
    # Heatmap zeichnen
    im = ax.imshow(matrix, cmap=custom_cmap, aspect='auto', vmin=-1, vmax=1)
    
    # Labels setzen
    ax.set_xticks(range(max_executions))
    ax.set_xticklabels([f"Exec {i+1}" for i in range(max_executions)])
    ax.set_yticks(range(len(test_names)))
    ax.set_yticklabels([f"Test {i+1}: {name[:20]}..." if len(name) > 20 else f"Test {i+1}: {name}" 
                       for i, name in enumerate(test_names)])
    
    # Status-Symbole hinzufügen
    for i in range(len(test_names)):
        for j in range(max_executions):
            if j < len(results[str(i+1)]["executions"]):
                value = matrix[i][j]
                if value == 1:
                    text = "✓"
                    color = "white"
                elif value == -1:
                    text = "✗"
                    color = "white"
                else:
                    text = "○"
                    color = "black"
                ax.text(j, i, text, ha='center', va='center', color=color, fontsize=12, fontweight='bold')
    
    ax.set_title(f'Test-Coverage-Matrix - App-Version {version}', fontsize=14, fontweight='bold')
    ax.set_xlabel('Ausführungen', fontweight='bold')
    ax.set_ylabel('Tests', fontweight='bold')
    
    # Custom Legend
    legend_elements = [
        plt.Rectangle((0,0),1,1, facecolor='#2ECC71', label='✓ Bestanden'),
        plt.Rectangle((0,0),1,1, facecolor='#E74C3C', label='✗ Fehlgeschlagen'),
        plt.Rectangle((0,0),1,1, facecolor='#95A5A6', label='○ Nicht getestet')
    ]
    ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1, 0.5))
    
    plt.tight_layout()
    plt.savefig(f"{output_dir}/test_coverage_v{version}.png", dpi=300, bbox_inches='tight')
    plt.close()

def create_title_with_info(fig, title, app_info):
    """Erstellt einen Titel mit App-Informationen."""
    if app_info:
        android_build = app_info.get("android_build")
        ios_build = app_info.get("ios_build")
        test_date = app_info.get("test_date")
        
        subtitle = []
        if android_build and android_build != "unbekannt":
            subtitle.append(f"Android Build: {android_build}")
        if ios_build and ios_build != "unbekannt":
            subtitle.append(f"iOS Build: {ios_build}")
        if test_date:
            subtitle.append(f"Testdatum: {test_date}")
        
        if subtitle:
            fig.suptitle(title, fontsize=16, fontweight='bold')
            plt.figtext(0.5, 0.98, " | ".join(subtitle), ha="center", fontsize=10)
        else:
            fig.suptitle(title, fontsize=16, fontweight='bold')
    else:
        fig.suptitle(title, fontsize=16, fontweight='bold')

def main():
    parser = argparse.ArgumentParser(description="Visualisiert Testergebnisse")
    parser.add_argument("--file", help="Pfad zur JSON-Datei mit Testergebnissen")
    args = parser.parse_args()
    
    results, version, app_info = load_results(args.file)
    if results:
        print(f"Visualisiere Testergebnisse aus App-Version {version}...")
        visualize_results(results, version, app_info)
    else:
        print("Keine Ergebnisse zum Visualisieren gefunden.")

if __name__ == "__main__":
    main()
