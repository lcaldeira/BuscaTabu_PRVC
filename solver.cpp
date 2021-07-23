/*=======================================================================
 * 					PROBLEMA DE ROTEAMENTO DE VEÍCULOS
 *				TRABALHO 1 - SOLUÇÃO ATRAVÉS DO MÉTODO GULOSO
 *=======================================================================
 *
 *		link das instâncias de teste
 *		http://neo.lcc.uma.es/vrp/vrp-instances/capacitated-vrp-instances/
 *
 * 	compilação e execução:
 *		clear && g++ solver.cpp -o solver -std=c++17 && ./solver A-n32-k5
 *		clear && g++ solver.cpp -o solver -std=c++17 && time ./solver A-n32-k5
 *		clear && g++ solver.cpp -o solver -std=c++17 && valgrind ./solver A-n32-k5
 *
 * 	compilação e execução com registro em arquivo:
 *		clear && g++ solver.cpp -DREG_ENABLED -o solver -std=c++17 && ./solver A-n32-k5
 *
 *	rotina de testes:
 *		time bash routine.sh
 *		time bash routine.sh > data/result && nano data/result
 */
#include <CppLib/DS/List.h>
#include <CppLib/DS/Vector.h>
#include <CppLib/DS/Matrix.h>
#include <CppLib/UT/Sorting.h>
#include <fstream>
#include <cmath>
#include <chrono>

using namespace std;
using namespace std::chrono;
using namespace DataStructure;
using namespace Sorting;

class Vertex;
class Timer;
class LogType;
class RouteSetting;

typedef bool (*NgbrIterator)(RouteSetting*,RouteSetting*,int&,int&);
typedef RouteSetting* (*TSFunction)(RouteSetting*,int,int);

string INSTANCE;
Matrix<int> DIST;
Vector<Vertex> PLACE;
List<LogType> LOG = List<LogType>();
Vector<NgbrIterator> ITER = Vector<NgbrIterator>(3);

int N, M, MAX_CAP, MIN_COST, MIN_QT;
bool REV_ROUTE = false;
float INIT_TIME;
int MAX_REP = 100;
int DEPOT = 0;

//////////////////////////////////////////////////////////////////////

class Vertex
{
public:
	int x,y,demand;
	
	Vertex() : x(0), y(0), demand(0) {}
	Vertex(int x0, int y0, int d0) : x(x0), y(y0), demand(d0) {}
	
	bool operator==(Vertex& v)
	{
		return (x==v.x && y==v.y && demand==v.demand);
	}
};

class Timer
{
protected:
	steady_clock::time_point t1, t2;
public:
	Timer(){}
	void start(){ t1 = steady_clock::now(); }
	void stop(){ t2 = steady_clock::now(); }
	int measure(){ return (int) duration_cast<microseconds>(t2-t1).count(); }
};

class LogType
{
public:
	char bt_variant;
	int param_n;
	int param_t;
	int param_max;
	bool rev_route;
	int value_in;
	int value_out;
	int qt_ciclos;
	float time_s;
	
	LogType(){}
	
	LogType(char btv, int t, int m, int vin)
	{
		param_n = N;
		bt_variant = btv;
		param_t = t;
		param_max = m;
		value_in = vin;
		rev_route = REV_ROUTE;
		REV_ROUTE = !REV_ROUTE;
		qt_ciclos = 0;
		time_s = 0;
	}
	
	bool operator==(LogType& l){ return false; }
};

class RouteSetting
{
public:
	Vector<int> route;	//cidades na ordem de entrega (rotas em sequência)
	Vector<int> split;	//vetor de "separação" - contém o índice da cidade final de cada rota
	int cost = 0;
	int qt = 0;
	
	RouteSetting(){}
	
	RouteSetting(int m)
	{
		route = Vector<int>(m);
		split = Vector<int>(m);
	}
	
	RouteSetting& operator=(const RouteSetting& rs)
	{
		this->route = rs.route;
		this->split = rs.split;
		this->cost = rs.cost;
		this->qt = rs.qt;
		
		return *this;
	}

	bool isBetterThan(RouteSetting* rs)
	{
		return (cost < rs->cost || (cost == rs->cost && qt < rs->qt));
	}
	
	std::string toString()
	{
		std::stringstream ss;
		size_t i, j, idx0, idx1 = -1;	
		
		for(i=0; i<qt; i++)
		{
			idx0 = idx1 + 1;
			idx1 = split[i];
			ss << "Route #" << (i+1) << ':';
			
			for(j=idx0; j<=idx1; j++)
				ss << ' ' << route[j];
			
			ss << '\n';
		}
		
		ss << "cost " << cost << '\n';
		return ss.str();
	}
	
	void toFile(string file_name)
	{
		ofstream file(file_name);
	
		if(!file.is_open())
		{
			cout << "ERRO: arquivo '" << file_name << "'não pode ser aberto.\n";
			exit(0);
		}
		
		file << toString();
		file.close();
	}
	
	void printDetails()
	{
		size_t i, j, idx0, idx1 = -1;
		int c, d, c_sum = 0, d_sum = 0;	
		
		for(i=0; i<qt; i++)
		{
			idx0 = idx1 + 1;
			idx1 = split[i];
			//custo e demanda
			c = DIST[DEPOT][ route[idx0] ];
			d = 0;
			
			cout << "Route #" << (i+1) << ':';
			
			for(j=idx0; j<=idx1; j++)
			{
				d += PLACE[ route[j] ].demand;
				if(j != idx1)
					c += DIST[ route[j] ][ route[j+1] ];
				cout << ' ' << route[j];
			}
			
			c += DIST[ route[idx1] ][DEPOT];
			c_sum += c;
			d_sum += d;
			cout << "\n * c = " << c << "\n * d = " << d << "\n";
		}
		
		cout << "total cost: " << c_sum << '\n';
		cout << "total demand: " << d_sum << '\n';
	}
};

//////////////////////////////////////////////////////////////////////

void readFile(string file_name);
void registerLog(string file_name);
void calcFitness(RouteSetting *rs);
void loadBalancing(RouteSetting *rs);
void partialRouting(RouteSetting *rs);
void reverseRoute(RouteSetting* rs, RouteSetting* rv);
bool iteratorSwap(RouteSetting* seed, RouteSetting* ngbr, int& i, int& j);
bool iterator2opt(RouteSetting* seed, RouteSetting* ngbr, int& i, int& j);
bool iterator3opt(RouteSetting* seed, RouteSetting* ngbr, int& i, int& j);

RouteSetting* greedyMethod();
RouteSetting* localSearch(RouteSetting* rs);
RouteSetting* choose(RouteSetting* rs1, RouteSetting* rs2);
RouteSetting* tabuSearch1(RouteSetting* rs, int t, int max_iter);
RouteSetting* tabuSearch2(RouteSetting* rs, int t, int max_iter);
RouteSetting* tabuSearch3(RouteSetting* rs, int t, int max_iter);

//////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	//leitura do arquivo de entrada
	if(argc <= 1)
	{
		cout << "ERRO: parâmetros insuficientes, informe o nome do arquivo de entrada.\n";
		exit(0);
	}
	
	MIN_QT = 0;
	for(int i=0; argv[1][i] != '\0'; i++)
		if(argv[1][i] == 'k')
		{
			for(int j=i+1; argv[1][j] != '\0'; j++)
				MIN_QT = (MIN_QT * 10) + (argv[1][j] - '0');
			break;
		}
	INSTANCE = (string) argv[1];
	readFile("instances/" + INSTANCE + ".vrp");
	
	//preparação
	int cost=0, k=0, R=4;
	randomSeed();
	Timer timer;
	int exec_time[R];
	RouteSetting *rs[R], *rsAux;
	RouteSetting *rsRev = new RouteSetting(M);
	TSFunction tsFunc[] = {tabuSearch1, tabuSearch2, tabuSearch3};
	int paramT[] = {N/3, N/6, N/9};
	int paramK[] = {4*N, 3*N, 2*N};
	int qt_param = 3;
	
	//seleção de vizinhanças a percorrer (em ordem)
	ITER.pushBack(iterator2opt);
	ITER.pushBack(iteratorSwap);
	
	//solução inicial
	timer.start();
	
	rsAux = greedyMethod();
	partialRouting(rsAux);
	rs[0] = localSearch(rsAux);
	delete rsAux;
	
	timer.stop();
	exec_time[0] = timer.measure();
	INIT_TIME = timer.measure() / 1.0e6;
	
	//busca tabu - variantes 1,2,3
	for(int i=0; i < 3; i++)
	{
		rsAux = rs[0];
		timer.start();
		
		for(int j=0; j < qt_param; j++)
		{
			reverseRoute(rsAux, rsRev);
			rsAux = choose(tsFunc[i](rsAux, paramT[j], paramK[j]),
						   tsFunc[i](rsRev, paramT[j], paramK[j]));
			
			rsAux = (j ? choose(rsAux,rs[i+1]) : rsAux);
			rs[i+1] = rsAux;
		}
		
		timer.stop();
		exec_time[i] = timer.measure();
	}
	
	//repetição para a variante 3
	for(int i=1; i < MAX_REP; i++)
	{
		rsAux = rs[0];
		timer.start();
		
		for(int j=0; j < qt_param; j++)
		{
			reverseRoute(rsAux, rsRev);
			rsAux = choose(tabuSearch3(rsAux, paramT[j], paramK[j]),
						   tabuSearch3(rsRev, paramT[j], paramK[j]));
			
			rsAux = choose(rsAux, rs[3]);
			rs[3] = rsAux;
		}
	
		timer.stop();
		exec_time[R-1] += timer.measure();
	}
	
	//impressão dos resultados
	long unsigned int sum_us = 0;
	float sec, delta;
	
	cout << "| " << setw(10) << left << argv[1];
	cout << "| " << setw(5) << right << MIN_COST << " |";
	
	for(int i=0; i < R; i++)
	{
		if(i != 1)
			cout << setw(7) << rs[i]->cost;
		else
			cout << " | " << setw(4) << rs[i]->cost;
		
		sum_us += exec_time[i];
		if(rs[i]->cost <= rs[k]->cost)
			k = i;
	}
	
	//tempo gasto e diferença percentual
	sec = (float) (sum_us/1.0e6);
	delta = rs[k]->cost/float(MIN_COST) - 1.0;
	
	cout << " |" << setw(6) << floor(sec) << '.' << left;
	cout << setw(3) << ceil((sec - floor(sec))*1000) << " s" << right;
	cout << " |" << setw(5) << round(1e4*delta)/100.0 << " % |";
	cout << (delta <=0 ? "<- " : "   ") << k << endl;
	
	#ifdef REG_ENABLED
	registerLog("logs/log-" + INSTANCE);
	rs[k]->toFile("outputs/out-" + INSTANCE);
	//rs[k]->printDetails();
	#endif
	
	for(int i=0; i < R; i++)
		delete rs[i];
	delete rsRev;
	
	return 0;
}


//////////////////////////////////////////////////////////////////////


void calcFitness(RouteSetting *rs)
{
	//calcular custo
	rs->split.clear();
	rs->cost = DIST[DEPOT][ rs->route[0] ];
	int cap = PLACE[ rs->route[0] ].demand;
	int i, j, k;

	for(i=1; i<M; i++)
	{
		j = rs->route[i];
		k = rs->route[i-1];
		cap += PLACE[j].demand;
		//capacidade dentro do limite o máximo
		if(cap <= MAX_CAP)
			rs->cost += DIST[k][j];
		else
		{
			//registrar fim de rota
			rs->split.pushBack(i-1);
			//contabilizar novo veículo e passagem pelo depósito
			cap = PLACE[j].demand;
			rs->cost += DIST[k][DEPOT] + DIST[DEPOT][j];
		}
	}
	//registro da última parada
	k = rs->route[M-1];
	rs->cost += DIST[k][DEPOT];
	rs->split.pushBack(M-1);
	rs->qt = rs->split.getSize();
}

void loadBalancing(RouteSetting *rs)
{
	bool flag;
	size_t i, j, k, idx0, idx1 = -1;
	int demand[rs->qt], new_dist, old_dist;
	
	//cálculo da demanda de cada rota
	for(i=0; i < rs->qt; i++)
	{
		idx0 = idx1+1;
		idx1 = rs->split[i];
		demand[i] = 0;
		
		for(j=idx0; j<=idx1; j++)
			demand[i] += PLACE[ rs->route[j] ].demand;
	}
	
	//redistribuição da carga visando equilíbrio e diminuição do custo da rota
	for(i = rs->qt-1; i>0; i--)
	{
		do
		{
			flag = false;
			j = rs->split[i-1];
			k = rs->route[j];
			
			//	(...) -> [idx0] -> [k] -> DEPOT -> [idx1] -> (...)
			idx0 = rs->route[j-1];
			idx1 = rs->route[j+1];
			old_dist = DIST[idx0][k] + DIST[k][DEPOT] + DIST[DEPOT][idx1];
			new_dist = DIST[idx0][DEPOT] + DIST[DEPOT][k] + DIST[k][idx1];
			
			if(new_dist < old_dist && demand[i] + PLACE[k].demand <= MAX_CAP)
			{
				flag = true;
				rs->split[i-1]--;
				demand[i] += PLACE[k].demand;
				demand[i-1] -= PLACE[k].demand;
				rs->cost += new_dist - old_dist;
			}
		}
		while(flag);
	}
}

void reverseRoute(RouteSetting* rs, RouteSetting* rev)
{
	*rev = *rs;
	rev->route.reverse();
	calcFitness(rev);
	loadBalancing(rev);
}

void partialRouting(RouteSetting* rs)
{
	int i, j, k, p, q=0;
	int id0, id1, id2;
	
	while(q < rs->qt)
	{
		//ajuste dos índices para as extremidades da rota
		i = (q ? rs->split[q-1]+1 : 0);
		j = rs->split[q++];
		id0 = DEPOT;
		//reconstrução gulosa parcial
		while(j > i+1)
		{
			p = i;
			id2 = rs->route[p];
			//busca à esquerda pelo mais próximo (quanto ao custo)
			for(k = i+1; k<=j; k++)
			{
				id1 = rs->route[k];
				if(float (DIST[id0][id1] + DIST[id1][DEPOT]) / PLACE[id1].demand <
				   float (DIST[id0][id2] + DIST[id2][DEPOT]) / PLACE[id2].demand)
					p = k;
			}
			//troca e avanço dos índices
			if(i != p)
				rs->route.swap(i,p);
			id0 = rs->route[i++];
		}
	}
	
	calcFitness(rs);
	loadBalancing(rs);
}

bool iteratorSwap(RouteSetting* seed, RouteSetting* ngbr, int& i, int& j)
{
	static bool state = false;
	
	//início da máquina de estados
	if(!state && seed)
	{
		i = 0;
		j = 1;
		state = true;
	}
	//avanço no espaço de busca
	else if(seed)
	{
		i++;
		j++;
		if(j == M)
			state = false;
	}
	else
		state = false;

	//geração do vizinho
	if(state && ngbr)
	{
		ngbr->route = seed->route;
		ngbr->route.swap(i, j);
		calcFitness(ngbr);
		loadBalancing(ngbr);
	}
	return state;
}

bool iterator2opt(RouteSetting* seed, RouteSetting* ngbr, int& i, int& j)
{
	static bool state = false;
	
	//início da máquina de estados
	if(!state && seed)
	{
		i = 0;
		j = 2;
		state = true;
	}
	//avanço no espaço de busca
	else if(seed)
	{
		j++;
		if(j == M)
		{
			i++;
			j=i+2;
			if(i+2 == M)
				state = false;
		}
	}
	else
		state = false;

	//geração do vizinho
	if(state && ngbr)
	{
		ngbr->route = seed->route;
		//                  |                   : middle point (j-i+1)
		//    |<----+-------x-------+---->|     : iter. range (i,j)
		//          |               |           : swap points (i+p, j-p)
		//          V               V
		// ~~[i]~~[i+p]~~[j-i+1]~~[j-p]~~[j]~~  : route
		for(int p = 0; p < (j-i+1)>>1; p++)
			ngbr->route.swap(i+p, j-p);
		calcFitness(ngbr);
		loadBalancing(ngbr);
	}
	return state;
}

bool iterator3opt(RouteSetting* seed, RouteSetting* ngbr, int& i, int& k)
{
	static bool state = false;
	static int j;
	
	//início da máquina de estados
	if(!state && seed)
	{
		i = 0;
		j = 2;
		k = 5;
		state = true;
	}
	//avanço no espaço de busca
	else if(seed)
	{
		//índice (k)
		k++;
		if(k == M)
		{
			//índice (j)
			j++;
			k=j+3;
			if(j+3 == M)
			{
				//índice (i)
				i++;
				j=i+2;
				k=j+3;
				if(i+5 == M)
					state = false;
			}
		}
	}
	else
		state = false;

	//geração do vizinho
	if(state && ngbr)
	{
		ngbr->route = seed->route;
		// de (i) até (j)
		for(int p = 0; p < (j-i+1)>>1; p++)
			ngbr->route.swap(i+p, j-p);
		// de (j+1) até (k)
		for(int p = 0; p < (k-j)>>1; p++)
			ngbr->route.swap(j+1+p, k-p);
		calcFitness(ngbr);
		loadBalancing(ngbr);
	}
	return state;
}

RouteSetting* greedyMethod()
{
	RouteSetting* rs = new RouteSetting(M);
	int min_id=0, cap, id, last_target, next_target;
	int cost, best_cost;
	
	//lista de locais a visitar (ignorando o depósito i=0)
	auto target = List<int>();
	Node<int> *node;
	
	for(int i=1; i<N; i++)
		target.pushBack(i);
	
	while(target.getSize() > 0)
	{
		//criação de uma nova rota de veículo
		cap = MAX_CAP;
		next_target = DEPOT;
		
		//montagem da rota
		do
		{
			node = target.nthNode(0);
			last_target = next_target;
			best_cost = 0;
			
			//para cada alvo a ser atingido, ver qual possui mais prioridade (menos valor)
			//dentro da capacidade de carga restante (cap) no veículo
			for(int i=0; i<target.getSize(); i++)
			{
				id = node->value;
				cost = DIST[last_target][id];
				
				if(PLACE[id].demand <= cap && (cost < best_cost || !best_cost))
				{
					best_cost = cost;
					next_target = id;
				}
				node = node->next();
			}
			
			//submeter o alvo escolhido na rota e removê-lo da lista de alvos
			//diminuir a capacidade restante no veículo
			if(next_target != last_target)
			{
				rs->route.pushBack(next_target);
				target.erase(target.indexOf(next_target));
				cap -= PLACE[next_target].demand;
				rs->cost += DIST[last_target][next_target];
			}
		}
		while(next_target != last_target);
		
		rs->cost += DIST[last_target][DEPOT]; //custo da volta para o depósito
		rs->split.pushBack(rs->route.getSize()-1); //registro de fim de rota
		rs->qt++;
	}
	return rs;
}

RouteSetting* localSearch(RouteSetting* rs)
{
	int i,j;
	RouteSetting *best = new RouteSetting(M);
	RouteSetting *ngbr = new RouteSetting(M);
	
	*best = *rs;
	
	//busca na vizinhança swap
	while(iteratorSwap(rs,ngbr,i,j))
		if(ngbr->isBetterThan(best))
			*best = *ngbr;
	
	//busca na vizinhança 2-opt
	while(iterator2opt(rs,ngbr,i,j))
		if(ngbr->isBetterThan(best))
			*best = *ngbr;
	
	delete ngbr;
	return best;
}

RouteSetting* choose(RouteSetting* rs1, RouteSetting* rs2)
{
	if(rs1->isBetterThan(rs2))
	{
		delete rs2;
		return rs1;
	}
	else
	{
		delete rs1;
		return rs2;
	}
}

//////////////////////////////////////////////////////////////////////
/*
	-> incrementa o contador k a cada ciclo de buscas 
	-> não efetua o roteamento parcial
*/
RouteSetting* tabuSearch1(RouteSetting* rs, int t, int max_iter)
{
	bool flag;
	int t1, t2;
	int i, j, k=0, n;
	int tabu[M];
	RouteSetting* ngbr = new RouteSetting(M);		//solução vizinha
	RouteSetting* curr = new RouteSetting(M);		//solução corrente
	RouteSetting* next = new RouteSetting(M);		//próxima solução
	RouteSetting* best = new RouteSetting(M);		//melhor solução
	LogType log = LogType('1',t,max_iter,rs->cost);
	Timer timer;
	
	//início do cronômetro
	timer.start();
	
	//valores iniciais
	*next = *rs;
	*best = *rs;
	
	for(i=0; i < M; i++)
		tabu[i] = 0;
	
	while(k < max_iter)
	{
		k++;
		flag = true;
		log.qt_ciclos++;
		
		for(n=0; n < ITER.getSize(); n++)
		{
			//iniciar a solução corrente
			*curr = *next;
			//varredura na vizinhança através da função de iteração
			while(ITER[n](curr,ngbr,i,j))
				if((!tabu[i] && !tabu[j] && ngbr->isBetterThan(next)) || 
					ngbr->isBetterThan(best) || flag)
				{
					flag = false;
					*next = *ngbr;
					t1 = i;
					t2 = j;
					
					if(ngbr->isBetterThan(best))
					{
						*best = *ngbr;
						k = 0;
					}
				}
		}
		
		//atualização da lista tabu
		for(i=0; i < M; i++)
			if(tabu[i] > 0)
				tabu[i]--;
		tabu[t1] = tabu[t2] = t;
		
		//incremento dos contadores entre t1 e t2
		for(i=t1+1; i<t2; i++)
			tabu[i]++;
	}
	
	
	//fim do cronômetro
	timer.stop();
	
	#ifdef REG_ENABLED
	//registro de parâmetros
	log.value_out = best->cost;
	log.time_s = timer.measure() / 1.0e6;
	LOG.pushBack(log);
	#endif
	
	delete curr;
	delete next;
	delete ngbr;
	
	return best;
}

//////////////////////////////////////////////////////////////////////
/*
	-> incrementa o contador k a cada ciclo de buscas
	-> efetua o roteamento parcial como função de penalidade em next
*/
RouteSetting* tabuSearch2(RouteSetting* rs, int t, int max_iter)
{
	bool flag;
	int t1, t2, t3;
	int i, j, k=0, n;
	int tabu[M];
	RouteSetting* ngbr = new RouteSetting(M);		//solução vizinha
	RouteSetting* curr = new RouteSetting(M);		//solução corrente
	RouteSetting* next = new RouteSetting(M);		//próxima solução
	RouteSetting* best = new RouteSetting(M);		//melhor solução
	LogType log = LogType('2',t,max_iter,rs->cost);
	Timer timer;
	
	//início do cronômetro
	timer.start();
	
	//valores iniciais
	*next = *rs;
	*best = *rs;
	partialRouting(next);
	
	for(i=0; i < M; i++)
		tabu[i] = 0;
	
	while(k < max_iter)
	{
		k++;
		flag = false;
		log.qt_ciclos++;
		
		for(n=0; n < ITER.getSize(); n++)
		{
			//iniciar a solução corrente e a próxima candidata
			*curr = *next;
			partialRouting(next);
			//varredura na vizinhança através da função de iteração
			while(ITER[n](curr,ngbr,i,j))
				if((!tabu[i] && !tabu[j] && ngbr->isBetterThan(next)) || 
					ngbr->isBetterThan(best))
				{
					flag = true;
					*next = *ngbr;
					t1 = i;
					t2 = j;
					
					if(next->isBetterThan(best))
					{
						*best = *next;
						k = 0;
					}
				}
		}
		
		//atualização da lista tabu
		for(i=0; i < M; i++)
			if(tabu[i] > 0)
				tabu[i]--;
		
		if(flag)
		{
			tabu[t1] = tabu[t2] = t;		
			//incremento dos contadores entre t1 e t2
			for(i=t1+1; i<t2; i++)
				tabu[i]++;
		}
	}
	
	//fim do cronômetro
	timer.stop();
	
	#ifdef REG_ENABLED
	//registro de parâmetros
	log.value_out = best->cost;
	log.time_s = timer.measure() / 1.0e6;
	LOG.pushBack(log);
	#endif
	
	delete curr;
	delete next;
	delete ngbr;
	
	return best;
}

//////////////////////////////////////////////////////////////////////
/*
	-> incrementa o contador k a cada ciclo de busca
	-> efetua o roteamento parcial conforme a solução se torna estagnada
	-> interrompe o ciclo de buscas caso ocorra atualização de best
*/
RouteSetting* tabuSearch3(RouteSetting* rs, int t, int max_iter)
{
	bool flag;
	int t1, t2;
	int i, j, k=0, n;
	int tabu[M];
	RouteSetting* ngbr = new RouteSetting(M);		//solução vizinha
	RouteSetting* curr = new RouteSetting(M);		//solução corrente
	RouteSetting* next = new RouteSetting(M);		//próxima solução
	RouteSetting* best = new RouteSetting(M);		//melhor solução
	LogType log = LogType('3',t,max_iter,rs->cost);
	Timer timer;
	
	//início do cronômetro
	timer.start();
	
	//valores iniciais
	*next = *rs;
	*best = *rs;
	partialRouting(next);
	
	for(i=0; i < M; i++)
		tabu[i] = 0;
	
	while(k < max_iter)
	{
		k++;
		flag = true;
		log.qt_ciclos++;
		
		for(n=0; n < ITER.getSize(); n++)
		{
			//iniciar a solução corrente
			*curr = *next;
			//chance de efetuar o roteamento parcial (aumenta conforme a estagnação)
			if(randomNum(0,max_iter) <= k)
			{
				partialRouting(next);
				flag = true;
			}
			//varredura na vizinhança através da função de iteração
			while(ITER[n](curr,ngbr,i,j))
				if((!tabu[i] && !tabu[j] && ngbr->isBetterThan(next)) || 
					ngbr->isBetterThan(best) || 
					(flag && randomNum(0,best->cost) > ngbr->cost - best->cost))
				{
					//registro do movimento efetuado
					*next = *ngbr;
					flag = false;
					t1 = i;
					t2 = j;
					
					if(ngbr->isBetterThan(best))
					{
						*best = *ngbr;
						k = 0;
						//reset do iterador e interrupção do ciclo
						ITER[n](nullptr,nullptr,i,j);
						break;
					}
				}
		}
		
		//busca na vizinhança 3-opt
		if(k == max_iter)
		{
			*curr = *best;
			
			while(iterator3opt(curr,ngbr,i,j))
				if(ngbr->isBetterThan(best))
				{
					*next = *ngbr;
					*best = *ngbr;
					t1 = i;
					t2 = j;
					k = 0;
					//penalidade
					t++;
					//reset do iterador e interrupção do ciclo
					iterator3opt(nullptr,nullptr,i,j);
					break;
				}
		}
		
		//atualização da lista tabu e do contador de iterações
		for(i=0; i < M; i++)
			if(tabu[i])
				tabu[i]--;
		tabu[t1] = tabu[t2] = t;
		
		//incremento dos contadores entre t1 e t2 com chance de 50%
		for(i=t1+1; i<t2; i++)
			if(randomNum(0,1) > 0.5)
				tabu[i]++;
	}
	
	//fim do cronômetro
	timer.stop();
	
	#ifdef REG_ENABLED
	//registro de parâmetros
	log.value_out = best->cost;
	log.time_s = timer.measure() / 1.0e6;
	LOG.pushBack(log);
	#endif
	
	delete curr;
	delete next;
	delete ngbr;
	
	return best;
}

/*RouteSetting* tabuSearch2(RouteSetting* rs, int t, int max_iter)
{
	float prob;
	int t1, t2;
	int i, j, k = 0, n;
	int tabu[M];
	long int freq[N];
	RouteSetting* ngbr = new RouteSetting(M);		//solução vizinha
	RouteSetting* curr = new RouteSetting(M);		//solução corrente
	RouteSetting* next = new RouteSetting(M);		//próxima solução
	RouteSetting* best = new RouteSetting(M);		//melhor solução
	RouteSetting* ptr = nullptr;						//ponteiro rsAuxiliar
	Vector<NgbrIterator> ITER = Vector<NgbrIterator>(2);
	
	//início do cronômetro
	Timer::start();
	
	//valores iniciais da solução corrente
	*next = *rs;
	*best = *rs;
	
	for(i=0; i < M; i++)
	{
		tabu[i] = 0;
		freq[i] = 0;
	}
	freq[M] = 0;
	
	//seleção de vizinhanças a percorrer
	ITER.pushBack(iterator2opt);
	ITER.pushBack(iteratorSwap);
	
	while(k++ < max_iter)
	{
		for(n=0; n < ITER.getSize(); n++)
		{
			//iniciar a solução corrente e a próxima candidata
			*curr = *next;
			partialRouting(next);
			//varredura na vizinhança através da função de iteração
			ptr = ngbr;
			while(ITER[n](curr,ptr,i,j))
			{
				if(ptr)
					if((!tabu[i] && !tabu[j] && ngbr->isBetterThan(next)) || 
						ngbr->isBetterThan(best))
					{
						*next = *ngbr;
						t1 = i;
						t2 = j;
			
						if(ngbr->isBetterThan(best))
						{
							*best = *next;
							k=0;
						}
					}
				//probabilidade de procurar novos vizinhos
				//(aumenta à medida que certas regiões se tornam recorrentes)
				prob = randomNum(0,freq[M]);
				ptr = (prob <=  freq[t1]+freq[t2] ? ngbr : nullptr);
			}
		}
		
		//atualização da lista tabu
		for(i=0; i < M; i++)
			if(tabu[i] > 0)
				tabu[i]--;
		tabu[t1] = tabu[t2] = t;
		//atualização da lista de frequencia
		freq[t1] += t;
		freq[t2] += t;
		freq[M] += t;
	}
	
	//fim do cronômetro
	Timer::stop();
	best->us = Timer::measure() + rs->us;
	
	delete curr;
	delete next;
	delete ngbr;
	
	return best;
}*/


//////////////////////////////////////////////////////////////////////


void readFile(string file_name)
{
	ifstream file(file_name);
	string line;
	float dx, dy;
	int x, y;
	
	if(!file.is_open())
	{
		cout << "ERRO: arquivo '" << file_name << "' não encontrado.\n";
		exit(0);
	}
	
	//preenchimento do grafo
	getline(file,line);
	getline(file,line);
	
	for(int i=0; i<line.size()-1; i++)
		if(line[i]=='v' && line[i+1]=='a')
		{
			i+=7;
			MIN_COST=0;
			while(line[i]>='0' && line[i]<='9' && i<line.size())
			{
				MIN_COST = (MIN_COST*10) + (line[i]-'0');
				i++;
			}
			break;
		}
	
	getline(file,line);
	getline(file,line);
	
	N = stoi(line.substr(12,line.size()-12));	// num de vértices
	M = N-1;									// num de vértices sem o depósito
	
	getline(file,line);
	getline(file,line);
	
	MAX_CAP = stoi(line.substr(11,line.size()-11));
	getline(file,line);	//pula a linha NODE_COORD_SECTION
	
	PLACE = Vector<Vertex>(N);
	DIST = Matrix<int>(N,N);
	
	for(int i=1; i<=N; i++)
	{
		file >> i >> x >> y;
		PLACE.pushBack(Vertex(x,y,0));
	}
	
	file >> line;	//pula a linha DEMAND_SECTION
	
	for(int i=1; i<=N; i++)
	{
		file >> i >> PLACE[i-1].demand;
		DIST[i-1][i-1] = 0;
		for(int j=i; j<N; j++)
		{
			dx = PLACE[i-1].x - PLACE[j].x;
			dy = PLACE[i-1].y - PLACE[j].y;
			DIST[i-1][j] = round(sqrt(pow(dx,2) + pow(dy,2)));
			DIST[j][i-1] = DIST[i-1][j];
		}
	}
	
	file.close();
}

void registerLog(string file_name)
{
	ofstream file(file_name);

	if(!file.is_open())
	{
		cout << "ERRO: arquivo '" << file_name << "'não pode ser aberto.\n";
		exit(0);
	}
	
	float err;
	auto node = LOG.nthNode(0);
	size_t cont = 0, size = LOG.getSize();
	
	file << fixed;
	
	while(!LOG.isBaseNode(node) && cont < size)
	{
		//parâmetros
		file << left << INSTANCE << " \t";
		file << "BT-" << node->value.bt_variant << right;
		file << " " << setw(4) << node->value.param_n;
		file << " " << setw(4) << node->value.param_t;
		file << " " << setw(6) << node->value.param_max;
		file << " " << setw(4) << (node->value.rev_route ? 1 : 0);
		file << " " << setw(6) << MIN_COST;
		//estatísticas
		file << " " << setw(6) << node->value.value_in;
		file << " " << setw(6) << node->value.value_out;
		file << " " << setw(6) << node->value.qt_ciclos;
		file << " " << setw(8) << setprecision(4) << (node->value.time_s + INIT_TIME);
		//erro percentual
		err = node->value.value_out/float(MIN_COST) - 1.0;
		file << " " << setw(8) << setprecision(3) << err << endl;
		//avanço do ponteiro
		node = node->next();
		cont++;
	}
	file.close();
}
