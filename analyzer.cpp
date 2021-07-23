/*-----------------------------------------------------------------------
 *						ARQUIVO DE PRÉ-PROCESSAMENTO DE DADOS
 *-----------------------------------------------------------------------
 *
 *		lê os dados compilados do log de saída e gera pares ordenados para
 *		serem plotados
 *
 *		curvas:
 *			1) erro (médio, mínimo, máximo) por método X instância de teste
 *			2) ?
 *
 *		compilação e execução:
 *			clear && g++ -std=c++17 analyzer.cpp -o analyzer && ./analyzer
 *
 */
#include <CppLib/DS/Vector.h>
#include <fstream>
#include <cmath>
#include <climits>

using namespace std;
using namespace DataStructure;

typedef struct
{
	int min = 0, max = 0, sum = 0, precision = 0;
	int samples = 0, hits = 0, cycles = 0;
	double time = 0;
	int n, k;
} DataPoint;

bool operator==(const DataPoint& ep1, const DataPoint& ep2)
{
	return (ep1.min == ep2.min && ep1.max == ep2.max && ep1.sum == ep2.sum && 
				ep1.samples == ep2.samples && ep1.hits == ep2.hits && 
			 	ep1.precision == ep2.precision && ep1.cycles == ep2.cycles && 
			 	ep1.time == ep2.time && ep1.n == ep2.n && ep1.k == ep2.k);
}

bool operator!=(const DataPoint& ep1, const DataPoint& ep2) 
{
	return !(ep1 == ep2);
}

int N = 74;
auto best = Vector<int>(N);
auto instance = Vector<string>(N);
auto ts_v1 = Vector<DataPoint>(N);
auto ts_v2 = Vector<DataPoint>(N);
auto ts_v3 = Vector<DataPoint>(N);
auto direct = Vector<DataPoint>(N * (2+150)*6 / 2);
auto revers = Vector<DataPoint>(N * (2+150)*6 / 2);

void export_data(string fname, Vector<DataPoint> vec);

int main()
{
	//variáveis de leitura
	int num, cont = -1, k_value, cont_t=0;
	float last_t[2];
	double flt;
	string str;
	DataPoint *pTS_v, *pInType;
	
	//abertura do arquivo
	ifstream file("data/raw_log");
	
	if(!file.is_open())
	{
		cout << "ERRO: arquivo 'raw_log' não encontrado.\n";
		exit(0);
	}
	
	//leitura e descarte do cabeçalho
	getline(file, str);
	
	//leitura dos valores
	while(!file.eof())
	{
		//nome da instância
		file >> str;
		if(file.eof())
			break;
		
		if(instance.isEmpty() || instance[ instance.getSize()-1 ] != str)
		{
			cont++;
			instance.pushBack(str);
			ts_v1.pushBack(DataPoint());
			ts_v2.pushBack(DataPoint());
			ts_v3.pushBack(DataPoint());
			k_value = stoi(str.substr( str.find('k') + 1 ));
			last_t[0] = last_t[1] = 0;
			
			//cout << '[' << cont << ']' << str << endl;
		}
		
		//método avaliado
		file >> str;
		pTS_v = (str == "BT-1" ? &ts_v1[cont] : 
					(str == "BT-2" ? &ts_v2[cont] : 
											&ts_v3[cont]));
		
		//leitura dos valores de desempenho
		file >> num; // [N]
		pTS_v->n = num;
		pTS_v->k = k_value;
		
		file >> str; // [T]
		file >> str; // [MAX]
		
		file >> str; // [REV]
		file >> num; // [BEST]
		if(best.getSize() == cont)
			best.pushBack(num);
		
		file >> str; // [INPUT]
		file >> num; // [OUTPUT]
		
		pTS_v->samples++;
		pTS_v->sum += num;
		if(num > pTS_v->max)
			pTS_v->max = num;
		if(num < pTS_v->min || pTS_v->min == 0)
		{
			pTS_v->min = num;
			pTS_v->precision = 1;
		}
		else if(num == pTS_v->min)
			pTS_v->precision++;
		if(num <= best[cont])
			pTS_v->hits++;
		
		file >> num; // [CYCLES]
		pTS_v->cycles += num;
		
		file >> flt; // [TIME]
		if(flt < 0)
			flt = ((flt * 1.0e6) - INT_MIN) / 1.0e6;
		pTS_v->time += flt;
		
		//correção do tempo acumulado
		if(pTS_v->samples % 6 > 1)
			pTS_v->time -= last_t[cont_t];
		cont_t = !((((pTS_v->samples-1) % 12) >> 1) % 2);
		last_t[!cont_t] *= pTS_v->samples % 2;
		last_t[!cont_t] += flt;
		
		file >> str; // [ERROR]
	}
	
	file.close();
	
	export_data("data/statistics_ts1", ts_v1);
	export_data("data/statistics_ts2", ts_v2);
	export_data("data/statistics_ts3", ts_v3);
	
	return 0;
	
	for(int i=0; i < N; i++)
	{
		cout << instance[i] << "\t";
		cout << ts_v1[i].hits << "\t";
		cout << ts_v1[i].precision << "\t";
		cout << ts_v1[i].samples << "\n";
	}
}

void export_data(string fname, Vector<DataPoint> vec)
{
	ofstream file(fname);
	int avg;

	if(!file.is_open())
	{
		cout << "ERRO: arquivo '" << fname << "'não pode ser aberto.\n";
		exit(0);
	}
	
	file << "INSTANCE\tN\tK\tBEST\tMIN\tMAX\tAVG\tHITS\t";
	file << "MIN_ERR\tMAX_ERR\tAVG_ERR\tPREC\t";
	file << "AVG_CYC\tAVG_TIME\tTOTAL_TIME\tSAMPLES\n";
	
	file << fixed;
	
	for(int i=0; i < N; i++)
	{
		vec[i].sum = round(vec[i].sum / (float) vec[i].samples);
	
		file << instance[i] << '\t';
		file << vec[i].n << '\t';
		file << vec[i].k << '\t';
		file << best[i] << '\t';
		file << vec[i].min << '\t';
		file << vec[i].max << '\t';
		file << vec[i].sum << '\t';
		file << setprecision(1);
		file << (vec[i].hits * 100.0 / vec[i].samples) << '\t';
		
		file << setprecision(5);
		file << (vec[i].min / (float) best[i] - 1.0) << '\t';
		file << (vec[i].max / (float) best[i] - 1.0) << '\t';
		file << (vec[i].sum / (float) best[i] - 1.0) << '\t';
		file << setprecision(1);
		file << (vec[i].precision * 100.0 / vec[i].samples) << '\t';
		
		file << setprecision(3);
		file << (vec[i].cycles / vec[i].samples) << '\t';
		file << (vec[i].time / (float) vec[i].samples) << '\t';
		file << (vec[i].time) << '\t';
		file << vec[i].samples << '\n';
	}
	
	file.close();
}
