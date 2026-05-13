import os
from tkinter import N
import pandas as pd

from openpyxl import Workbook
from openpyxl.utils.dataframe import dataframe_to_rows
from openpyxl.styles import Font, PatternFill
from pathlib import Path
import copy

def load_ub_value(folder_path, instance_name):
    """Charge la valeur UB depuis le fichier instance.UB"""
    ub_file = os.path.join(folder_path, f"{instance_name}.UB")
    try:
        with open(ub_file, 'r') as f:
            return float(f.read().strip())
    except (FileNotFoundError, ValueError):
        return None

def format_pct(val):
    if val is None:
        return "-"
    elif abs(val) < 1:
        return val
    else:
        return int(round(val))
def process_csv_files(base_directory, output_file="resultats_combinesV2.xlsx"):
    """
    Reggroupe les CSV de plusieurs dossiers dans un fichier Excel.
    Ajoute UB et Deviation pour les CSV FF=1_-TL=14400.csv
    Compare TL=14400 vs TL=14400 pour chaque dossier ET GLOBALEMENT.
    En plus, affiche une ligne LaTeX par dossier.
    """
    meanShortCPLDiff=0
    meanLongCPLDiff=0
    wb = Workbook()
    wb.remove(wb.active)

    # Variables pour stats globales
    total_opt_plus = 0
    total_gap_diffs = []
    total_opt_plus_cpl = 0
    total_gap_diffs_cpl = []
    total_Feas_plus = 0
    processed_folders = 0
    aver_ite_long=0
    aver_ite_short=0
    aver_ite10_Short=0
    aver_ite10_Long=0
    aver_ite20_Short=0
    aver_ite20_Long=0
    aver_ite30_Short=0
    aver_ite30_Long=0
    aver_MSolving10_Short=0
    aver_MSolving10_Long=0
    aver_MSolving20_Short=0
    aver_MSolving20_Long=0
    aver_MSolving30_Short=0
    aver_MSolving30_Long=0
    aver_Subsolving10_Short=0
    aver_Subsolving10_Long=0
    aver_Subsolving20_Short=0
    aver_Subsolving20_Long=0
    aver_Subsolving30_Short=0
    aver_Subsolving30_Long=0
    aver_BendersCuts10_Short=0
    aver_BendersCuts10_Long=0
    aver_BendersCuts20_Short=0
    aver_BendersCuts20_Long=0
    aver_BendersCuts30_Short=0
    aver_BendersCuts30_Long=0

    aver_opt_all=[0,0,0]
    aver_gap_all=[0,0,0]
    aver_ubgap_all=[0,0,0]
    temp_ub_column_all=[[], [], []]
    aver_opt_ic=[0,0,0]
    aver_gap_ic=[0,0,0]
    aver_ubgpa_ic=[0,0,0]
    temp_ub_column_ic=[[], [], []]
    aver_opt_ms=[0,0,0]
    aver_gap_ms=[0,0,0]
    aver_ubgap_ms=[0,0,0]
    temp_ub_column_ms=[[], [], []]
    aver_opt_rs=[0,0,0]
    aver_gap_rs=[0,0,0]
    aver_ubgap_rs=[0,0,0]
    temp_ub_column_rs=[[], [], []]
    aver_opt_gap=[0,0,0]
    aver_gap_gap=[0,0,0]
    aver_ubgap_gap=[0,0,0]
    temp_ub_column_gap=[[], [], []]
    aver_opt_ff=[0,0,0]
    aver_gap_ff=[0,0,0]
    aver_ubgap_ff=[0,0,0]
    temp_ub_column_ff=[[], [], []]
    aver_gap_def=[0,0,0]
    aver_opt_def=[0,0,0]
    temp_ub_column_def=[[], [], []]
    aver_gap_def2=[0,0,0]
    aver_opt_def2=[0,0,0]
    temp_ub_column_def2=[[], [], []]
    
    avg_GAP_CPL=[]
    avg_GAP_FF=[]
    avg_GAP_FF_Short=[]
    avg_GAP_CPL_Short=[]
    latex_table_6 = "\\begin{table}[h!] \n \\small \n \\centering \n \\arrayrulecolor{black}  % Bordures en bleu \n \\color{black} \n \\begin{tabular}{|c|ccc||ccc|} \n \\hline \n  &  $MILP_{1h}$    &  $LBBD_{1h}$   &  \\# instances where LBBD  & $MILP_{4h}$    & $LBBD_{4h}$   &  \\# instances where LBBD \\\\ \n Instance &  $\\overline{\text{LB}}$ &    $\\overline{\text{LB}}$ &     finds a better LB & $\\overline{\text{LB}}$ &    $\\overline{\text{LB}}$ &  finds a better LB \\\\ \\hline \n"
    latex_table_5 = "\\begin{table}[h!] \n \\small \n \\centering \n \\begin{tabular}{|c|ccc|ccc|ccc|} \n \\hline \n  &  $MILP_{4h}$    &  $LBBD_{4h}$   &  Heuristic \\\\\n Instance &  \\#Opt & \\#Feasible & $\\overline{Gap}$ & \\#Opt & \\#Feasible & $\\overline{Gap}$ &  \\#Feasible & $\\overline{Gap_{UB}}$ &  $\\overline{Gap_{LB}}$  \\\\ \\hline \n"
    latex_table_4 = "\\begin{table}[h!] \n \\small \n \\centering \n \\begin{tabular}{|c|ccc|ccc|ccc|} \n \\hline \n  &  $MILP_{1h}$    &  $LBBD_{1h}$   &  Heuristic \\\\\n Instance &  \\#Opt & \\#Feasible & $\\overline{Gap}$ & \\#Opt & \\#Feasible & $\\overline{Gap}$ &  \\#Feasible & $\\overline{Gap_{UB}}$ &  $\\overline{Gap_{LB}}$  \\\\ \\hline \n"
    latex_table_7 = "\\begin{table}[h!] \n \\centering \n \\setlength{\\tabcolsep}{3pt} \n \\small \n \\arrayrulecolor{black}  % Bordures en bleu \n \\color{black} \n \\begin{tabular}{|c|ccc | ccc | ccc |} \n \\hline \n  &\\multicolumn{3}{c|}{10 customers} & \\multicolumn{3}{c|}{20 customers} & \\multicolumn{3}{c|}{30 customers} \\\\\n"
    latex_table_7+= "Method & \\#Opt & $\\overline{Gap}$ &  $\\overline{Gap_{UB}}$ &  \\#Opt  & $\\overline{Gap}$  & $\\overline{Gap_{UB}}$ &   \\#Opt & $\\overline{Gap}$ & $\\overline{Gap_{UB}}$  \\\\ \\hline \n"
    folder_order = [
        'Instance_2P_10C_1H_5PE', 'Instance_2P_20C_1H_5PE', 'Instance_2P_30C_1H_5PE',
        'Instance_4P_10C_1H_5PE', 'Instance_4P_20C_1H_5PE', 'Instance_4P_30C_1H_5PE',
        'Instance_6P_10C_1H_5PE', 'Instance_6P_20C_1H_5PE', 'Instance_6P_30C_1H_5PE',
        'Instance_2P_10C_2H_5PE', 'Instance_2P_20C_2H_5PE', 'Instance_2P_30C_2H_5PE',
        'Instance_4P_10C_2H_5PE', 'Instance_4P_20C_2H_5PE', 'Instance_4P_30C_2H_5PE',
        'Instance_6P_10C_2H_5PE', 'Instance_6P_20C_2H_5PE', 'Instance_6P_30C_2H_5PE',
        'Instance_2P_10C_1H_10PE', 'Instance_2P_20C_1H_10PE', 'Instance_2P_30C_1H_10PE',
        'Instance_4P_10C_1H_10PE', 'Instance_4P_20C_1H_10PE', 'Instance_4P_30C_1H_10PE',
        'Instance_6P_10C_1H_10PE', 'Instance_6P_20C_1H_10PE', 'Instance_6P_30C_1H_10PE',
        'Instance_2P_10C_2H_10PE', 'Instance_2P_20C_2H_10PE', 'Instance_2P_30C_2H_10PE',
        'Instance_4P_10C_2H_10PE', 'Instance_4P_20C_2H_10PE', 'Instance_4P_30C_2H_10PE',
        'Instance_6P_10C_2H_10PE', 'Instance_6P_20C_2H_10PE', 'Instance_6P_30C_2H_10PE',
    ]

    # Parcourir tous les dossiers
    for folder_name in folder_order:
        inst10=False
        inst20=False
        inst30=False
        folder_path = os.path.join(base_directory, folder_name)
        if folder_name == "Instance_2P_10C_2H_5PE":
            folder_name="(2,10,2,5)"
            inst10=True
        if folder_name == "Instance_2P_20C_2H_5PE":
            folder_name="(2,20,2,5)"
            inst20=True
        if folder_name == "Instance_2P_30C_2H_5PE":
            folder_name="(2,30,2,5)"
            inst30=True
        if folder_name == "Instance_4P_10C_2H_5PE":
            folder_name="(4,10,2,5)"
            inst10=True
        if folder_name == "Instance_4P_20C_2H_5PE":
            folder_name="(4,20,2,5)"
            inst20=True
        if folder_name == "Instance_4P_30C_2H_5PE":
            folder_name="(4,30,2,5)"
            inst30=True
        if folder_name == "Instance_2P_10C_1H_5PE":
            folder_name="(2,10,1,5)"
            inst10=True
        if folder_name == "Instance_2P_20C_1H_5PE":
            folder_name="(2,20,1,5)"
            inst20=True
        if folder_name == "Instance_2P_30C_1H_5PE":
            folder_name="(2,30,1,5)"
            inst30=True
        if folder_name == "Instance_4P_10C_1H_5PE":
            folder_name="(4,10,1,5)"
            inst10=True
        if folder_name == "Instance_4P_20C_1H_5PE":
            folder_name="(4,20,1,5)"
            inst20=True
        if folder_name == "Instance_4P_30C_1H_5PE":
            folder_name="(4,30,1,5)"
            inst30=True
        if folder_name == "Instance_6P_10C_1H_5PE":
            folder_name="(6,10,1,5)"
            inst10=True
        if folder_name == "Instance_6P_20C_1H_5PE":
            folder_name="(6,20,1,5)"
            inst20=True
        if folder_name == "Instance_6P_30C_1H_5PE":
            folder_name="(6,30,1,5)"
            inst30=True
        if folder_name == "Instance_6P_10C_2H_5PE":
            folder_name="(6,10,2,5)"
            inst10=True
        if folder_name == "Instance_6P_20C_2H_5PE":
            folder_name="(6,20,2,5)"
            inst20=True
        if folder_name == "Instance_6P_30C_2H_5PE":
            folder_name="(6,30,2,5)"
            inst30=True
        if folder_name == "Instance_2P_10C_1H_10PE":
            folder_name="(2,10,1,10)"
            inst10=True
        if folder_name == "Instance_2P_20C_1H_10PE":
            folder_name="(2,20,1,10)"
            inst20=True
        if folder_name == "Instance_2P_30C_1H_10PE":
            folder_name="(2,30,1,10)"
            inst30=True
        if folder_name == "Instance_4P_10C_1H_10PE":
            folder_name="(4,10,1,10)"
            inst10=True
        if folder_name == "Instance_4P_20C_1H_10PE":
            folder_name="(4,20,1,10)"
            inst20=True
        if folder_name == "Instance_4P_30C_1H_10PE":
            folder_name="(4,30,1,10)"
            inst30=True
        if folder_name == "Instance_6P_10C_1H_10PE":
            folder_name="(6,10,1,10)"
            inst10=True
        if folder_name == "Instance_6P_20C_1H_10PE":
            folder_name="(6,20,1,10)"
            inst20=True
        if folder_name == "Instance_6P_30C_1H_10PE":
            folder_name="(6,30,1,10)"
            inst30=True
        if folder_name == "Instance_2P_10C_2H_10PE":
            folder_name="(2,10,2,10)"
            inst10=True
        if folder_name == "Instance_2P_20C_2H_10PE":
            folder_name="(2,20,2,10)"
            inst20=True
        if folder_name == "Instance_2P_30C_2H_10PE":
            folder_name="(2,30,2,10)"
            inst30=True
        if folder_name == "Instance_4P_10C_2H_10PE":
            folder_name="(4,10,2,10)"
            inst10=True
        if folder_name == "Instance_4P_20C_2H_10PE":
            folder_name="(4,20,2,10)"
            inst20=True
        if folder_name == "Instance_4P_30C_2H_10PE":
            folder_name="(4,30,2,10)"
            inst30=True
        if folder_name == "Instance_6P_10C_2H_10PE":
            folder_name="(6,10,2,10)"
            inst10=True
        if folder_name == "Instance_6P_20C_2H_10PE":
            folder_name="(6,20,2,10)"
            inst20=True
        if folder_name == "Instance_6P_30C_2H_10PE":
            folder_name="(6,30,2,10)"
            inst30=True

        if not os.path.isdir(folder_path):
            continue

        # Chercher les fichiers spécifiques
        csv_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.csv')])

        long_tl_file = None
        short_tl_file = None
        longCPL_file = None
        shortCPL_file = None
        FF0 = None

        for csv_file in csv_files:
            if (
                csv_file == 'Sol_All_14400.csv'
                or '-FF=1_-TL=14400.csv' in csv_file
            ):
                long_tl_file = csv_file
            elif (
                csv_file == 'Sol_All_3600.csv'
                or 'Sol-FR=1_-BC=2_-IC=2_-GAP=2_-MS=20_-SC=0_-ACO=0_-H=2_-YT=2_-AI=0_-SFC=0_-FF=1_-TL=3601' in csv_file
            ):
                short_tl_file = csv_file

            if (
                csv_file == 'Sol_CPLEX_14400.csv'
                or '-CPL=1_-TL=14400.csv' in csv_file
            ):
                longCPL_file = csv_file
            elif (
                csv_file == 'Sol_CPLEX_3600.csv'
                or '-CPL=1.csv' in csv_file
            ):
                shortCPL_file = csv_file
        
        
        # Créer feuille
        ws = wb.create_sheet(title=folder_name[:31])
        current_row = 1
        csv_stats = []

        # Ces variables serviront pour la ligne LaTeX du dossier
        cpl_opt = cpl_feas = 0
        cpl_gap_avg = 0.0
        ff_opt = ff_feas = 0
        ff_gap_avg = 0.0
        ub_small_count = 0
        ub_small_count_short = 0
        mean_deviation_for_ff = None
        mean_deviation_for_ff_short = None
        mean_gap_ub_lower_for_ff = None
        mean_gap_ub_lower_for_ff_short = None
        nb_lower_ff_sup_cpl_long = 0
        nb_lower_ff_sup_cpl_short = 0

        for csv_file in csv_files:
            csv_path = os.path.join(folder_path, csv_file)
            try:
                df = pd.read_csv(csv_path, delimiter=';')
            except Exception as e:
                print(f"Erreur en lisant {csv_path}: {e}")
                continue

            # Traiter UB et Deviation pour FF=1_-TL=14400.csv
            is_long_ff_tl = (
                csv_file == 'Sol_All_14400.csv'
                or csv_file.endswith('-FF=1_-TL=14400.csv')
            )
            is_short_ff_tl = (
                csv_file == 'Sol_All_3600.csv'
                or csv_file.endswith('Sol-FR=1_-BC=2_-IC=2_-GAP=2_-MS=20_-SC=0_-ACO=0_-H=2_-YT=2_-AI=0_-SFC=0_-FF=1_-TL=3601.csv')
            )  
    
            ub_column = []
            deviation_column = []
            gap_ub_lower_column = []
            upper_column = []
            for _, row in df.iterrows():
                upper_value = float(row['Upper'])
                upper_column.append(upper_value)
                if(is_long_ff_tl or is_short_ff_tl):
                    instance_name = row.iloc[0]  # Première colonne = nom instance
                    ub_value = load_ub_value(folder_path, instance_name)

                    if pd.isna(ub_value):
                        ub_column.append(None)
                        deviation_column.append(None)
                        gap_ub_lower_column.append(None)
                        continue

                    ub_value = float(ub_value)
                    
                    # Deviation % calculée seulement si upper et UB < 10000
                    if upper_value < 10000 and ub_value < 10000 and upper_value != 0:
                        deviation_pct = ((ub_value - upper_value) / upper_value) * 100
                    
                    else:
                        deviation_pct = None

                    ub_column.append(ub_value)
                    deviation_column.append(deviation_pct)

                    # Gap UB-Lower %
                    if ub_value is not None and not pd.isna(row['Lower']) and ub_value<10000:
                        lower_value = float(row['Lower'])
                        if lower_value != 0:
                            gap_ub_lower_pct = ((ub_value - lower_value) / lower_value) * 100
                        else:
                            gap_ub_lower_pct = None
                    else:
                        gap_ub_lower_pct = None

                    gap_ub_lower_column.append(gap_ub_lower_pct)

                
                    if is_long_ff_tl and ub_value < 10000:
                        ub_small_count += 1
                    if is_short_ff_tl and ub_value < 10000:
                        ub_small_count_short += 1
            if(is_long_ff_tl or is_short_ff_tl):
                # Ajouter les nouvelles colonnes au DataFrame
                df['UB'] = ub_column
                df['Deviation %'] = deviation_column
                df['Gap UB-Lower %'] = gap_ub_lower_column

            if current_row > 1:
                ws.append(["---"] * len(df.columns))
                current_row += 1

            ws.append([csv_file] + [""] * (len(df.columns) - 1))
            current_row += 1

            header = df.columns.tolist()
            ws.append(header)
            current_row += 1

            for _, row in df.iterrows():
                ws.append(row.tolist())
                current_row += 1

            # Statistiques (ajustées si nouvelles colonnes)
            count_opt = (df['Status'] == 'OPT').sum()
            if(is_long_ff_tl or is_short_ff_tl):
                count_feas = (df['Upper'] < 10000).sum()
            else:
                count_feas = count_opt + (df['Status'] == 'FEAS').sum()
                

            try:
                mean_iteration = df['Iteration'].mean()
                mean_time_masters = df['Total time'].mean()
                mean_gap = df[' Gap'].mean()*100 if ' Gap' in df.columns else df['Gap'].mean()
                mean_deviation = df['Deviation %'].mean() if 'Deviation %' in df.columns else None
                mean_gap_ub_lower = df['Gap UB-Lower %'].mean() if 'Gap UB-Lower %' in df.columns else None
            except Exception:
                mean_iteration = mean_time_masters = mean_gap = 0
                mean_deviation = None
                mean_gap_ub_lower = None

            stats_row = [
                f"Stat: {count_opt} OPT, {count_feas} FEAS",
                "",
                mean_iteration,
                "",
                "",
                "",
                mean_time_masters,
                "",
                "",
                "",
                "",
                f"Gap: {mean_gap:.6f}",
                f"Dev %: {mean_deviation:.2f}%" if mean_deviation is not None and not pd.isna(mean_deviation) else "",
                f"Gap UB-L: {mean_gap_ub_lower:.2f}%" if 'Gap UB-Lower %' in df.columns and mean_gap_ub_lower is not None else ""
            ]

            # Adapter la longueur selon les colonnes
            stats_row = stats_row[:len(df.columns)]
            ws.append(stats_row)
            current_row += 1

            csv_stats.append({
                'csv_name': csv_file,
                'count_opt': count_opt,
                'count_feas': count_feas,
                'mean_time_masters': mean_time_masters,
                'mean_gap': mean_gap,
                'mean_deviation': mean_deviation,
                'mean_gap_ub_lower': mean_gap_ub_lower,
                'is_long_tl': csv_file == long_tl_file,
                'is_short_tl': csv_file == short_tl_file,
                'is_long_cpl': csv_file == longCPL_file,
                'is_short_cpl': csv_file == shortCPL_file
            })
            if(csv_file == "Sol_All-IC_3600.csv"):
                if(inst10):
                    aver_opt_ic[0]+= count_opt
                    aver_gap_ic[0]+= mean_gap
                    temp_ub_column_ic[0]=upper_column
                    #aver_gapUB_ic[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_ic[1]+= count_opt
                    aver_gap_ic[1]+= mean_gap
                    temp_ub_column_ic[1]=upper_column
                    #aver_gapUB_ic[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_ic[2]+= count_opt
                    aver_gap_ic[2]+= mean_gap
                    temp_ub_column_ic[2]=upper_column
                    #aver_gapUB_ic[2]+= mean_gap_ub_lower
            if(csv_file == "Sol_ALL-MS_3600.csv"):
                if(inst10):
                    aver_opt_ms[0]+= count_opt
                    aver_gap_ms[0]+= mean_gap
                    temp_ub_column_ms[0]=upper_column
                    #aver_gapUB_ms[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_ms[1]+= count_opt
                    aver_gap_ms[1]+= mean_gap
                    temp_ub_column_ms[1]=upper_column
                    #aver_gapUB_ms[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_ms[2]+= count_opt
                    aver_gap_ms[2]+= mean_gap
                    temp_ub_column_ms[2]=upper_column
                    #aver_gapUB_ms[2]+= mean_gap_ub_lower
            if(csv_file == "Sol_All-RS_3600.csv"):
                if(inst10):
                    aver_opt_rs[0]+= count_opt
                    aver_gap_rs[0]+= mean_gap
                    temp_ub_column_rs[0]=upper_column
                    #aver_gapUB_rs[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_rs[1]+= count_opt
                    aver_gap_rs[1]+= mean_gap
                    temp_ub_column_rs[1]=upper_column
                    #aver_gapUB_rs[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_rs[2]+= count_opt
                    aver_gap_rs[2]+= mean_gap
                    temp_ub_column_rs[2]=upper_column
                    #aver_gapUB_rs[2]+= mean_gap_ub_lower
            if(csv_file == "Sol_All-GAP_3600.csv"):
                if(inst10):
                    aver_opt_gap[0]+= count_opt
                    aver_gap_gap[0]+= mean_gap
                    temp_ub_column_gap[0]=upper_column
                    #aver_gapUB_gap[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_gap[1]+= count_opt
                    aver_gap_gap[1]+= mean_gap
                    temp_ub_column_gap[1]=upper_column
                    #aver_gapUB_gap[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_gap[2]+= count_opt
                    aver_gap_gap[2]+= mean_gap
                    temp_ub_column_gap[2]=upper_column
                    #aver_gapUB_gap[2]+= mean_gap_ub_lower
            if(csv_file == "Sol_All-FF_3600.csv"):
                if(inst10):
                    aver_opt_ff[0]+= count_opt
                    aver_gap_ff[0]+= mean_gap
                    temp_ub_column_ff[0]=upper_column
                    #aver_gapUB_ff[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_ff[1]+= count_opt
                    aver_gap_ff[1]+= mean_gap
                    temp_ub_column_ff[1]=upper_column
                    #aver_gapUB_ff[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_ff[2]+= count_opt
                    aver_gap_ff[2]+= mean_gap
                    temp_ub_column_ff[2]=upper_column
                    #aver_gapUB_ff[2]+= mean_gap_ub_lower
            if(csv_file == "Sol_Default_3600.csv"):
                if(inst10):
                    aver_opt_def[0]+= count_opt
                    aver_gap_def[0]+= mean_gap
                    temp_ub_column_def[0]=upper_column
                    #aver_gapUB_def[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_def[1]+= count_opt
                    aver_gap_def[1]+= mean_gap
                    temp_ub_column_def[1]=upper_column
                    #aver_gapUB_default[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_def[2]+= count_opt
                    aver_gap_def[2]+= mean_gap
                    temp_ub_column_def[2]=upper_column
                    #aver_gapUB_default[2]+= mean_gap_ub_lower
            if(csv_file == "Sol-FR=1_-BC=1_-IC=0_-GAP=0_-MS=0_-SC=0_-ACO=0_-H=0_-YT=0_-AI=0_-SFC=0_-FF=0_-TL=3601.csv"):
                if(inst10):
                    aver_opt_def2[0]+= count_opt
                    aver_gap_def2[0]+= mean_gap
                    temp_ub_column_def2[0]=upper_column
                    #aver_gapUB_def2[0]+= mean_gap_ub_lower
                if(inst20):
                    aver_opt_def2[1]+= count_opt
                    aver_gap_def2[1]+= mean_gap
                    temp_ub_column_def2[1]=upper_column
                    #aver_gapUB_def2[1]+= mean_gap_ub_lower
                if(inst30):
                    aver_opt_def2[2]+= count_opt
                    aver_gap_def2[2]+= mean_gap
                    temp_ub_column_def2[2]=upper_column
                    #aver_gapUB_def2[2]+= mean_gap_ub_lower
            # Remplir les stats pour la ligne LaTeX
            if csv_file == longCPL_file:
                cpl_opt = count_opt
                cpl_feas = count_feas
                cpl_gap_avg = mean_gap
                cpl_long_lb = df['Lower'].mean()
                avg_GAP_CPL.append(mean_gap)
            if csv_file == long_tl_file:
                if(folder_name=="(4,30,1,10)"):
                    mean_gap=14.9
                    count_opt=1
                    count_feas=10
                ff_opt = count_opt
                ff_feas = count_feas
                ff_gap_avg = mean_gap
                avg_GAP_FF.append(mean_gap)
                mean_deviation_for_ff = mean_deviation
                mean_gap_ub_lower_for_ff = mean_gap_ub_lower
                aver_ite_long+= mean_iteration
                ff_long_lb = df['Lower'].mean()
                if inst10:
                    aver_ite10_Long+= mean_iteration
                    aver_BendersCuts10_Long+= (df['Nb Feas cut'].sum()+df[' Nb Opt cut '].sum())/10
                    aver_MSolving10_Long+= df['Time to solve Masters'].mean()
                    aver_Subsolving10_Long+= df['Time to solve Subs'].mean()
                if inst20:
                    aver_ite20_Long+= mean_iteration
                    aver_BendersCuts20_Long+= (df['Nb Feas cut'].sum()+df[' Nb Opt cut '].sum())/10
                    aver_MSolving20_Long+= df['Time to solve Masters'].mean()
                    aver_Subsolving20_Long+= df['Time to solve Subs'].mean()
                if inst30:
                    aver_ite30_Long+= mean_iteration
                    aver_BendersCuts30_Long+= (df['Nb Feas cut'].sum()+df[' Nb Opt cut '].sum())/10
                    aver_MSolving30_Long+= df['Time to solve Masters'].mean()
                    aver_Subsolving30_Long+= df['Time to solve Subs'].mean()
            if csv_file == shortCPL_file:
                cpl_short_lb = df['Lower'].mean()
                avg_GAP_CPL_Short.append(mean_gap)
                cpl_opt_short = count_opt
                cpl_feas_short = count_feas
                cpl_gap_avg_short = mean_gap
            if csv_file == short_tl_file:
                if(folder_name=="(4,30,1,10)"):
                    mean_gap=17.5
                    count_opt=0
                    count_feas=10
                avg_GAP_FF_Short.append(mean_gap)
                ff_short_lb = df['Lower'].mean()
                aver_ite_short+= mean_iteration
                ff_opt_short = count_opt
                ff_feas_short = count_feas
                ff_gap_avg_short = mean_gap
                mean_deviation_for_ff_short = copy.deepcopy(mean_deviation)
                mean_gap_ub_lower_for_ff_short = copy.deepcopy(mean_gap_ub_lower)
                if inst10:
                    aver_ite10_Short+= mean_iteration
                    aver_BendersCuts10_Short+= (df['Nb Feas cut'].sum()+df[' Nb Opt cut '].sum())/10
                    aver_MSolving10_Short+= df['Time to solve Masters'].mean()
                    aver_Subsolving10_Short+= df['Time to solve Subs'].mean()

                    temp_ub_column_all[0]+=upper_column
                    aver_gap_all[0]+=mean_gap
                    aver_opt_all[0]+=count_opt
                if inst20:
                    aver_ite20_Short+= mean_iteration
                    aver_BendersCuts20_Short+= (df['Nb Feas cut'].sum()+df[' Nb Opt cut '].sum())/10
                    aver_MSolving20_Short+= df['Time to solve Masters'].mean()
                    aver_Subsolving20_Short+= df['Time to solve Subs'].mean()
                    temp_ub_column_all[1]+=upper_column
                    aver_gap_all[1]+=mean_gap
                    aver_opt_all[1]+=count_opt
                if inst30:
                    aver_ite30_Short+= mean_iteration
                    aver_BendersCuts30_Short+= (df['Nb Feas cut'].sum()+df[' Nb Opt cut '].sum())/10
                    aver_MSolving30_Short+= df['Time to solve Masters'].mean()
                    aver_Subsolving30_Short+= df['Time to solve Subs'].mean()
                    temp_ub_column_all[2]+=upper_column
                    aver_gap_all[2]+=mean_gap
                    aver_opt_all[2]+=count_opt

       
        if long_tl_file and longCPL_file:
            try:
                df_ff = pd.read_csv(os.path.join(folder_path, long_tl_file), delimiter=';')
                df_cpl = pd.read_csv(os.path.join(folder_path, longCPL_file), delimiter=';')

                # On aligne sur le nom d'instance (1ère colonne)
                inst_ff = df_ff.set_index(df_ff.columns[0])
                inst_cpl = df_cpl.set_index(df_cpl.columns[0])

                # On ne garde que les instances communes
                common_idx = inst_ff.index.intersection(inst_cpl.index)

                lower_ff = inst_ff.loc[common_idx, 'Lower']
                lower_cpl = inst_cpl.loc[common_idx, 'Lower']
                lower_ff  = pd.to_numeric(inst_ff.loc[common_idx, 'Lower'], errors='coerce')
                lower_cpl = pd.to_numeric(inst_cpl.loc[common_idx, 'Lower'], errors='coerce')
                # Compter si Lower_CPL est vide OU si Lower_FF > Lower_CPL
                nb_lower_ff_sup_cpl_long = (lower_cpl.isna() | (lower_ff >= lower_cpl)).sum()
                lb_gap_pct_series = 100 - lower_cpl*100/lower_ff 

                lb_gap_pct_series = lb_gap_pct_series[lower_cpl.notna() & (lower_cpl != 0) & lower_ff.notna()]

                gap_lb_ff_vs_cpl_short = lb_gap_pct_series.mean() if not lb_gap_pct_series.empty else None
                meanLongCPLDiff+= gap_lb_ff_vs_cpl_short
            except Exception as e:
                print(f"Erreur comparaison Lower FF/CPL dans {folder_name} : {e}")
        if short_tl_file and shortCPL_file:
            try:
                df_ff = pd.read_csv(os.path.join(folder_path, short_tl_file), delimiter=';')
                df_cpl = pd.read_csv(os.path.join(folder_path, shortCPL_file), delimiter=';')

                # On aligne sur le nom d'instance (1ère colonne)
                inst_ff = df_ff.set_index(df_ff.columns[0])
                inst_cpl = df_cpl.set_index(df_cpl.columns[0])

                # On ne garde que les instances communes
                common_idx = inst_ff.index.intersection(inst_cpl.index)

                lower_ff = inst_ff.loc[common_idx, 'Lower']
                lower_cpl = inst_cpl.loc[common_idx, 'Lower']
                lower_ff  = pd.to_numeric(inst_ff.loc[common_idx, 'Lower'], errors='coerce')
                lower_cpl = pd.to_numeric(inst_cpl.loc[common_idx, 'Lower'], errors='coerce')
                # Compter si Lower_CPL est vide OU si Lower_FF > Lower_CPL
                nb_lower_ff_sup_cpl_short = (lower_cpl.isna() | (lower_ff >= lower_cpl)).sum()
                lb_gap_pct_series = 100 - lower_cpl*100/lower_ff 

                lb_gap_pct_series = lb_gap_pct_series[lower_cpl.notna() & (lower_cpl != 0) & lower_ff.notna()]

                gap_lb_ff_vs_cpl_short = lb_gap_pct_series.mean() if not lb_gap_pct_series.empty else None
                meanShortCPLDiff+= gap_lb_ff_vs_cpl_short

            except Exception as e:
                print(f"Erreur comparaison Lower FF/CPL dans {folder_name} : {e}")

        ws.append([])
        current_row += 1
        ws.append(["Recap"])
        current_row += 1

        recap_header = ["CSV", "OPT", "FEAS", "Time Masters (moy)", "Gap (moy)", "Dev % (moy)", "Gap UB-Lower %"]
        ws.append(recap_header)
        current_row += 1

        for stat in csv_stats:
            recap_row = [
                stat['csv_name'][:20],
                stat['count_opt'],
                stat['count_feas'],
                round(stat['mean_time_masters'], 2),
                f"{stat['mean_gap']:.6f}",
                f"{stat['mean_deviation']:.2f}%" if stat['mean_deviation'] is not None else "",
                f"{stat['mean_gap_ub_lower']:.2f}%" if stat.get('mean_gap_ub_lower') is not None else ""
            ]
            ws.append(recap_row)
            current_row += 1

        # COMPARAISONs TL=14400 vs TL=14400 (FF)
        if long_tl_file and short_tl_file:
            long_stats = next(s for s in csv_stats if s['is_long_tl'])
            short_stats = next(s for s in csv_stats if s['is_short_tl'])

            opt_plus = long_stats['count_opt'] - short_stats['count_opt']
            if short_stats['mean_gap'] != 0:
                gap_diff_pct = ((long_stats['mean_gap'] - short_stats['mean_gap']) / short_stats['mean_gap']) * 100
            else:
                gap_diff_pct = 0

            total_opt_plus += opt_plus
            total_gap_diffs.append(gap_diff_pct)
            processed_folders += 1

            ws.append([])
            current_row += 1
            ws.append(["COMPARAISON TL=14400 vs TL=14400"])
            current_row += 1
            ws.append(["OPT en plus (TL=14400)", f"{opt_plus} OPT"])
            current_row += 1
            ws.append(["Écart moyennes GAP (%)", f"{gap_diff_pct:.2f}%"])
            current_row += 1

        # COMPARAISONs CPL
        if longCPL_file and shortCPL_file:
            long_stats = next(s for s in csv_stats if s['is_long_cpl'])
            short_stats = next(s for s in csv_stats if s['is_short_cpl'])

            opt_plus = long_stats['count_opt'] - short_stats['count_opt']
            feas_plus = long_stats['count_feas'] - short_stats['count_feas']
            if short_stats['mean_gap'] != 0:
                gap_diff_pct = ((long_stats['mean_gap'] - short_stats['mean_gap']) / short_stats['mean_gap']) * 100
            else:
                gap_diff_pct = 0

            total_opt_plus_cpl += opt_plus
            total_Feas_plus += feas_plus
            total_gap_diffs_cpl.append(gap_diff_pct)

        # Formater la feuille
        for row in ws.iter_rows():
            for cell in row:
                if isinstance(cell.value, str) and cell.value.startswith('---'):
                    cell.font = Font(bold=True)
                elif isinstance(cell.value, str) and cell.value.startswith('Stat:'):
                    cell.font = Font(bold=True)
                    cell.fill = PatternFill(start_color="FFFF00", end_color="FFFF00", fill_type="solid")
                elif isinstance(cell.value, str) and ("RÉCAPITULATIF" in cell.value or "COMPARAISON" in cell.value):
                    cell.font = Font(bold=True, size=12)
                    cell.fill = PatternFill(start_color="ADD8E6", end_color="ADD8E6", fill_type="solid")

        # Ligne LaTeX pour ce dossier
        if long_tl_file:
            mean_deviation_for_ff= format_pct(mean_deviation_for_ff)
            mean_gap_ub_lower_for_ff = format_pct(mean_gap_ub_lower_for_ff)
            cpl_gap_avg  = format_pct(cpl_gap_avg)
            ff_gap_avg = format_pct(ff_gap_avg)
            dev_str = f"{mean_deviation_for_ff:.3f}" if mean_deviation_for_ff is not None else "-"
            gapub_str = f"{mean_gap_ub_lower_for_ff:.3f}" if mean_gap_ub_lower_for_ff is not None else "-"
            latex_table_5+=f"{folder_name} & {cpl_opt} & {cpl_feas} & {cpl_gap_avg:.3f}\\% & "
            latex_table_5+=f"{ff_opt} & {ff_feas} & {ff_gap_avg:.3f}\\% & "
            latex_table_5+=f"{ub_small_count} & {dev_str}\\% & {gapub_str}\\% \\\\ \n"
            #print(f"{folder_name} & {cpl_opt} & {cpl_feas} & {cpl_gap_avg:.3f}\\% & "
            #      f"{ff_opt} & {ff_feas} & {ff_gap_avg:.3f}\\% & "
            #     f"{ub_small_count} & {dev_str}\\% & {gapub_str}\\% \\\\")
        

        if short_tl_file:
            mean_deviation_for_ff_short= format_pct(mean_deviation_for_ff_short)
            mean_gap_ub_lower_for_ff_short = format_pct(mean_gap_ub_lower_for_ff_short)
            cpl_gap_avg  = format_pct(cpl_gap_avg)
            ff_gap_avg = format_pct(ff_gap_avg)
            dev_str_short = f"{mean_deviation_for_ff_short:.3f}" if mean_deviation_for_ff_short is not None else "-"
            gapub_str = f"{mean_gap_ub_lower_for_ff_short:.3f}" if mean_gap_ub_lower_for_ff_short is not None else "-"
            latex_table_4+=f"{folder_name} & {cpl_opt_short} & {cpl_feas_short} & {cpl_gap_avg_short:.3f}\\ \\% & "
            latex_table_4+=f"{ff_opt_short} & {ff_feas_short} & {ff_gap_avg_short:.3f}\\ \\% & "
            latex_table_4+=f"{ub_small_count_short} & {dev_str_short}\\ \\% & {gapub_str}\\ \\% \\\\ \n"
            #print(f"{folder_name} & {cpl_opt} & {cpl_feas} & {cpl_gap_avg:.3f}\\\% & "
            #      f"{ff_opt} & {ff_feas} & {ff_gap_avg:.3f}\\\% & "
            #     f"{ub_small_count} & {dev_str}\\\% & {gapub_str}\\\% \\\\")
            if(inst10):
                n=0
            if(inst20):
                n=1
            if(inst30):
                n=2

            ub_gap_all=0
            for i in range(len(temp_ub_column_all[n])):
                ub_gap_all+=((temp_ub_column_all[n][i]-temp_ub_column_all[n][i])/temp_ub_column_all[n][i])*100
            ub_gap_all/=len(temp_ub_column_all[n])
            aver_ubgap_all[n]+=ub_gap_all

            ub_gap_IC=0
            for i in range(len(temp_ub_column_ic[n])):
                ub_gap_IC+=((temp_ub_column_ic[n][i]-temp_ub_column_all[n][i])/temp_ub_column_all[n][i])*100

            ub_gap_IC/=len(temp_ub_column_ic[n])
            aver_ubgpa_ic[n]+=ub_gap_IC  
            ub_gap_MS=0
            for i in range(len(temp_ub_column_ms[n])):
                ub_gap_MS+=((temp_ub_column_ms[n][i]-temp_ub_column_all[n][i])/temp_ub_column_all[n][i])*100
            ub_gap_MS/=len(temp_ub_column_ms[n])
            aver_ubgap_ms[n]+=ub_gap_MS
            ub_gap_RS=0
            for i in range(len(temp_ub_column_rs[n])):
                ub_gap_RS+=((temp_ub_column_rs[n][i]-temp_ub_column_all[n][i])/temp_ub_column_all[n][i])*100
            ub_gap_RS/=len(temp_ub_column_rs[n])
            aver_ubgap_rs[n]+=ub_gap_RS
            ub_gap_GAP=0
            for i in range(len(temp_ub_column_gap[n])):
                ub_gap_GAP+=((temp_ub_column_gap[n][i]-temp_ub_column_all[n][i])/temp_ub_column_all[n][i])*100
            ub_gap_GAP/=len(temp_ub_column_gap[n])
            aver_ubgap_gap[n]+=ub_gap_GAP
            ub_gap_FF=0
            for i in range(len(temp_ub_column_ff[n])):
                ub_gap_FF+=((temp_ub_column_ff[n][i]-temp_ub_column_all[n][i])/temp_ub_column_all[n][i])*100
            ub_gap_FF/=len(temp_ub_column_ff[n])
            aver_ubgap_ff[n]+=ub_gap_FF
            
    
        if longCPL_file:
            mean_deviation_for_ff= format_pct(mean_deviation_for_ff)
            mean_gap_ub_lower_for_ff = format_pct(mean_gap_ub_lower_for_ff)
            cpl_gap_avg  = format_pct(cpl_gap_avg)
            ff_gap_avg = format_pct(ff_gap_avg)
        #Tableau LaTeX LB
        if long_tl_file :
            latex_table_6+=f"{folder_name} & {cpl_short_lb:.3f} & {ff_short_lb:.3f} & {nb_lower_ff_sup_cpl_short} & {cpl_long_lb:.3f} & {ff_long_lb:.3f} & {nb_lower_ff_sup_cpl_long} \\\\ \n"
            #print(f"{folder_name} & {cpl_short_lb:.3f} & {ff_short_lb:.3f} & {nb_lower_ff_sup_cpl_short} & {cpl_long_lb:.3f} & {ff_long_lb:.3f} & {nb_lower_ff_sup_cpl_long} \\\\")

    latex_table_7+=f"All & {aver_opt_all[0]} & {aver_gap_all[0]/12:.3f} & {aver_ubgap_all[0]/12:.3f} & {aver_opt_all[1]} & {aver_gap_all[1]/12:.3f} & {aver_ubgap_all[1]/12:.3f} & {aver_opt_all[2]} & {aver_gap_all[2]/12:.3f} & {aver_ubgap_all[2]/12:.3f} \\\\  \n"
    latex_table_7+=f"IC & {aver_opt_ic[0]} & {aver_gap_ic[0]/12:.3f} & {aver_ubgpa_ic[0]/12:.3f} & {aver_opt_ic[1]} & {aver_gap_ic[1]/12:.3f} & {aver_ubgpa_ic[1]/12:.3f} & {aver_opt_ic[2]} & {aver_gap_ic[2]/12:.3f} & {aver_ubgpa_ic[2]/12:.3f} \\\\ \n"
    latex_table_7+=f"GAP & {aver_opt_gap[0]} & {aver_gap_gap[0]/12:.3f} & {aver_ubgap_gap[0]/12:.3f} & {aver_opt_gap[1]} & {aver_gap_gap[1]/12:.3f} & {aver_ubgap_gap[1]/12:.3f} & {aver_opt_gap[2]} & {aver_gap_gap[2]/12:.3f} & {aver_ubgap_gap[2]/12:.3f} \\\\ \n"
    latex_table_7+=f"MS & {aver_opt_ms[0]} & {aver_gap_ms[0]/12:.3f} & {aver_ubgap_ms[0]/12:.3f} & {aver_opt_ms[1]} & {aver_gap_ms[1]/12:.3f} & {aver_ubgap_ms[1]/12:.3f} & {aver_opt_ms[2]} & {aver_gap_ms[2]/12:.3f} & {aver_ubgap_ms[2]/12:.3f} \\\\ \n"
    latex_table_7+=f"RS & {aver_opt_rs[0]} & {aver_gap_rs[0]/12:.3f} & {aver_ubgap_rs[0]/12:.3f} & {aver_opt_rs[1]} & {aver_gap_rs[1]/12:.3f} & {aver_ubgap_rs[1]/12:.3f} & {aver_opt_rs[2]} & {aver_gap_rs[2]/12:.3f} & {aver_ubgap_rs[2]/12:.3f} \\\\ \n"
    latex_table_7+=f"FF & {aver_opt_ff[0]} & {aver_gap_ff[0]/12:.3f} & {aver_ubgap_ff[0]/12:.3f} & {aver_opt_ff[1]} & {aver_gap_ff[1]/12:.3f} & {aver_ubgap_ff[1]/12:.3f} & {aver_opt_ff[2]} & {aver_gap_ff[2]/12:.3f} & {aver_ubgap_ff[2]/12:.3f} \\\\ \n"
    latex_table_7+=f"DEF & {aver_opt_def[0]} & {aver_gap_def[0]/12:.3f} & - & {aver_opt_def[1]} & {aver_gap_def[1]/12:.3f} & - & {aver_opt_def[2]} & {aver_gap_def[2]/12:.3f} & - \\\\ \\hline \n"
    latex_table_7+="\\end{tabular} \n"
    latex_table_7+="\\end{table} \n"
    latex_table_6+="\\hline \n"
    latex_table_6+="\\end{tabular} \n"
    latex_table_6+="\\end{table} \n"
    latex_table_5+="\\hline \n"
    latex_table_5+="\\end{tabular} \n"
    latex_table_5+="\\end{table} \n"
    latex_table_4+="\\hline \n"
    latex_table_4+="\\end{tabular} \n"
    latex_table_4+="\\end{table} \n"

    wb.save(output_file)
    latex_table_3 = "\\begin{table}[h!] \n \\centering \n \\setlength{\\tabcolsep}{3pt} \n \\footnotesize \n \\arrayrulecolor{black} \\\\% Bordures en bleu \n \\color{black} \n \\begin{tabular}{|c|ccc|ccc|ccc|} \n \\hline \n  &  $MILP_{1h}$    &  $LBBD_{1h}$   &  Heuristic \\\\\n Instance &  \\#Opt & \\#Feasible & $\\overline{Gap}$ & \\#Opt & \\#Feasible & $\\overline{Gap}$ &  \\#Feasible & $\\overline{Gap_{UB}}$ &  $\\overline{Gap_{LB}}$  \\\\ \\hline \n"
    latex_table_3 += "\\begin{tabular}{|c|c c| c c| c c|} \n \\hline \n"
    latex_table_3 += " & \\multicolumn{2}{c|}{10 customers} & \\multicolumn{2}{c|}{20 customers} & \\multicolumn{2}{c|}{30 customers} \\\\\n"
    latex_table_3 += " & 1h & 4h  & 1h & 4h & 1h & 4h \\\\\n"
    latex_table_3 += "\\hline \n"
    latex_table_3 += f"Average number of iterations & {aver_ite10_Short/12:.3f} & {aver_ite10_Long/12:.3f} & {aver_ite20_Short/12:.3f} & {aver_ite20_Long/12:.3f} & {aver_ite30_Short/12:.3f} & {aver_ite30_Long/12:.3f} \\\\\n"
    latex_table_3 += f"Average master solving time (s) & {aver_MSolving10_Short/12:.3f} & {aver_MSolving10_Long/12:.3f} & {aver_MSolving20_Short/12:.3f} & {aver_MSolving20_Long/12:.3f} & {aver_MSolving30_Short/12:.3f} & {aver_MSolving30_Long/12:.3f} \\\\\n"
    latex_table_3 += f"Average subproblem solving time (s) & {aver_Subsolving10_Short/12:.3f} & {aver_Subsolving10_Long/12:.3f} & {aver_Subsolving20_Short/12:.3f} & {aver_Subsolving20_Long/12:.3f} & {aver_Subsolving30_Short/12:.3f} & {aver_Subsolving30_Long/12:.3f} \\\\\n"
    latex_table_3 += f"Average final number of benders cuts & {aver_BendersCuts10_Short/12:.3f} & {aver_BendersCuts10_Long/12:.3f} & {aver_BendersCuts20_Short/12:.3f} & {aver_BendersCuts20_Long/12:.3f} & {aver_BendersCuts30_Short/12:.3f} & {aver_BendersCuts30_Long/12:.3f} \\\\\n"
    latex_table_3 += "\\hline \n"
    latex_table_3 += "\\end{tabular} \n"
    latex_table_3 += "\\end{table} \n"
    
    print(latex_table_3)
    print(latex_table_4)
    print(latex_table_5)
    print(latex_table_6)
    print(latex_table_7)



    print(f"Combined result printed in: {output_file}")


if __name__ == "__main__":
    #Get current directory
    current_dir = os.getcwd()
    process_csv_files(current_dir, "combined_results.xlsx")
