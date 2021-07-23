#########################################################
###    backup e preparação do ambiente de execução    ###
#########################################################
#bash backup_files.sh
g++ -std=c++17 solver.cpp -o solver #-DREG_ENABLED
g++ -std=c++17 analyzer.cpp -o analyzer

#########################################################
###    execução para todas as instancias de teste     ###
#########################################################

echo -e "+-----------+-------+--------+--------------------+-------------+--------+"
echo -e "| Instância | Ótimo | S_init |   Tabu (I-II-III)  | Tempo Total |  Erro  |"
echo -e "+-----------+-------+--------+--------------------+-------------+--------+"

for file in instances/A*; do
	file=${file##*/}	#retira tudo até a '/'
	file=${file%.*}		#retira tudo a partir do '.'
	./solver "$file"	#executa aquela instância
done

#wait
echo -e "+-----------+-------+--------+--------------------+-------------+--------+"

#########################################################
###	  rocessamento dos logs e geração dos gráficos    ###
#########################################################

echo -e "INSTANCE  METHOD    N    T    MAX   REV  BEST  INPUT  OUTPUT CYCLES    TIME    ERROR" > data/raw_log
for file in logs/*; do
	cat $file >> data/raw_log
done

#./analyzer
#python3 magic.py
