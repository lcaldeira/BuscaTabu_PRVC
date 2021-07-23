# coding=<utf8>
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

statisticsTS = [ 
	pd.read_csv('data/statistics_ts1', delimiter='\t', header=0),
	pd.read_csv('data/statistics_ts2', delimiter='\t', header=0),
	pd.read_csv('data/statistics_ts3', delimiter='\t', header=0)
];

###########################################################################
def absCostDistribution(subset, ybottom, ytop):

	dFilter = statisticsTS[0]['INSTANCE'].str.startswith(subset);
	
	dFrame = [ statisticsTS[0][dFilter].sort_values(by=['N']),
				  statisticsTS[1][dFilter].sort_values(by=['N']),
				  statisticsTS[2][dFilter].sort_values(by=['N']) ];
				  
	curves = [	dFrame[0].filter(['AVG','MIN','MAX']).values,
					dFrame[1].filter(['AVG','MIN','MAX']).values,
					dFrame[2].filter(['AVG','MIN','MAX']).values	];

	labels = dFrame[0].filter(['INSTANCE']).values[:,0];
	y_best = dFrame[0].filter(['BEST']).values[:,0];
	x_pos = np.arange(len(labels));
	colors = ['b','g','r'];
	bar_w = 0.22;

	plt.figure(figsize=(14,8));
	plt.margins(0.01, 0);
	plt.ylim([ybottom, ytop]);
	plt.xticks(x_pos+.5, labels, rotation=-45);

	plt.hlines(y_best, xmin=(x_pos-2*bar_w), xmax=(x_pos+2*bar_w), 
				label=("Ótimo Conhecido"), linestyle='solid', color='k', alpha=.5);

	for i in range(len(curves)):

		y_avg = curves[i][:,0];
		y_min = y_avg - curves[i][:,1];
		y_max = curves[i][:,2] - y_avg;
		y_lbl = 'Busca Tabu v' + str(i+1)
	
		plt.bar(x_pos + (i - 1)*bar_w, y_avg, bar_w, yerr=[y_min,y_max], 
			label=y_lbl, color=colors[i], ecolor=colors[i], alpha=.5);

	#plt.title("Custo do Roteamento Por Instância");
	plt.ylabel("Custo do Roteamento");
	plt.legend(loc='upper left');
	plt.tight_layout();
	plt.savefig('img/abs_cost_per_method_[' + subset + '].svg', dpi=300);
	#plt.show();

###########################################################################
def avgCycleAndHitPercent(fator_de_escala):
	
	dFrame = [	statisticsTS[0][ (statisticsTS[0]['MIN_ERR'] <= 0) ],
			  	statisticsTS[1][ (statisticsTS[1]['MIN_ERR'] <= 0) ],
			  	statisticsTS[2][ (statisticsTS[2]['MIN_ERR'] <= 0) ] ];
	
	curves = [	dFrame[0].filter(['N','AVG_CYC','HITS']).values,
				dFrame[1].filter(['N','AVG_CYC','HITS']).values,
				dFrame[2].filter(['N','AVG_CYC','HITS']).values	];

	colors = ['b','g','r'];

	plt.figure(figsize=(8,8));
	plt.margins(0.01, 0);

	for i in range(len(curves)):
		
		x_val = curves[i][:,0];
		y_val = curves[i][:,1];
		p_dim = curves[i][:,2] * fator_de_escala;
		
		plt.scatter(x_val, y_val, p_dim, 
			label=('Busca Tabu v' + str(i+1)), color=colors[i], alpha=.4);

	plt.xlabel("Número de Vértices");
	plt.ylabel("Número de Ciclos");
	plt.legend(loc='upper left');
	plt.tight_layout();
	#plt.savefig('img/hit_percent_per_method.svg', dpi=300);
	plt.show();

###########################################################################
def teste(fator_de_escala):
	
	dFrame = [ 	statisticsTS[0],
				statisticsTS[1],
				statisticsTS[2] ];
	
	curves = [	dFrame[0].filter(['N','K','MIN_ERR','PREC']).values,
				dFrame[1].filter(['N','K','MIN_ERR','PREC']).values,
				dFrame[2].filter(['N','K','MIN_ERR','PREC']).values	];

	colors = ['b','g','r'];

	plt.figure(figsize=(8,8));
	plt.margins(0.01, 0);
	#plt.xscale('log');
	#plt.yscale('log');

	for i in range(len(curves)):
		
		x_val = curves[i][:,0];
		y_val = curves[i][:,2];
		p_dim = curves[i][:,3] * fator_de_escala;
		
		plt.scatter(x_val, y_val, p_dim, 
			label=('Busca Tabu v' + str(i+1)), color=colors[i], alpha=.4);

	#plt.xlabel("Número de Vértices");
	#plt.ylabel("Número de Ciclos");
	plt.legend(loc='upper left');
	plt.tight_layout();
	#plt.savefig('img/hit_percent_per_method.svg', dpi=300);
	plt.show();

absCostDistribution('A', 600, 2200);
absCostDistribution('B', 500, 1800);
absCostDistribution('P', 200, 1100);
#avgCycleAndHitPercent(3);
#teste(3);
