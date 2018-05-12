/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (daniele.calisi@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
using namespace std;

#include <optionspp/optionsreader.h>
using namespace Options;

#include "basegeneticalgorithm.h"
using namespace GeneticAlgorithms;

ofstream ofs;

class KnapsackIndividual : public BaseIndividual<bool> {
public:
	char dummy;
	string toString() {
		string s = "";
		for (size_t i = 0; i < geneticRepresentation.size(); i++) {
			s += (geneticRepresentation[i] ? "1" : "0");
		}
		return s;
	}
};

class KnapsackProblem : public BaseGeneticAlgorithm<KnapsackIndividual, FitnessGreaterIsBetter> {
public:
	KnapsackProblem(size_t objectCount);
	void initialize(size_t populationSize);

	vector<uint> weights;
	vector<uint> values;
	uint maxWeight;
	
protected:
	FitnessGreaterIsBetter fitness(const KnapsackIndividual& individual);
	vector<KnapsackIndividual> selection();
	void reproduction(vector<KnapsackIndividual> selectedPopulation);
};

KnapsackProblem::KnapsackProblem(size_t objectCount)
{
	weights.resize(objectCount);
	values.resize(objectCount);
	maxWeight = 0;
	for (size_t i = 0; i < objectCount; i++) {
		weights[i] = rand() % 100;
		maxWeight += weights[i];
		values[i] = rand() % 100;
	}
	maxWeight /= 2;
	printf("Knapsack problem initialized: maxWeight = %d.\n", maxWeight);
}

void KnapsackProblem::initialize(size_t populationSize)
{
	population.clear();
	for (size_t i = 0; i < populationSize; i++) {
		KnapsackIndividual ind;
		ind.geneticRepresentation.resize(weights.size());
		for (size_t j = 0; j < weights.size(); j++) {
			ind.geneticRepresentation[j] = (rand() % 2 == 0);
		}
		population.push_back(ind);
	}
}

FitnessGreaterIsBetter KnapsackProblem::fitness(const KnapsackIndividual& individual)
{
	double f = 0.;
	uint w = 0;
	for (size_t i = 0; i < individual.geneticRepresentation.size(); i++) {
		if (individual.geneticRepresentation[i]) {
			w += weights[i];
			f += values[i];
		}
	}
	return (w <= maxWeight ? f : 0.);
}

vector<KnapsackIndividual> KnapsackProblem::selection()
{
	printf("Current population: %d individuals.\n", population.size());
	multimap<FitnessGreaterIsBetter, KnapsackIndividual*> m;
	for (size_t i = 0; i < population.size(); i++) {
		m.insert(make_pair(fitness(population[i]), &(population[i])));
	}
	vector<KnapsackIndividual> ret;
	for (multimap<FitnessGreaterIsBetter, KnapsackIndividual*>::iterator it = m.begin(); it != m.end(); ++it) {
		ret.push_back(*(it->second));
	}
	printf("Selected %d individual for reproduction.\n", ret.size());
	return ret;
}

void KnapsackProblem::reproduction(vector<KnapsackIndividual> selectedPopulation)
{
	vector<KnapsackIndividual> newPopulation;
	newPopulation.push_back(selectedPopulation[0]);	// the best individual of previous population
	// crossover
	size_t crossoverInd = 0;
	for (size_t i = 0; i < selectedPopulation.size()/2; i++) {
		KnapsackIndividual& ind1 = selectedPopulation[i];
		KnapsackIndividual& ind2 = selectedPopulation[i+1];
		KnapsackIndividual child;
		child.geneticRepresentation.resize(ind1.geneticRepresentation.size());
		for (size_t j = 0; j < child.geneticRepresentation.size(); j++) {
			child.geneticRepresentation[j] = (rand() % 2 == 0 ?
				ind1.geneticRepresentation[j] :
				ind2.geneticRepresentation[j]);
		}
		newPopulation.push_back(child);
		crossoverInd++;
	}
	// mutation
	size_t mutatedInd = 0;
	uint i = 0;
	while (newPopulation.size() < population.size()) {
		KnapsackIndividual& ind = (i < selectedPopulation.size() ? selectedPopulation[i] : selectedPopulation[0]);
		KnapsackIndividual mutated;
		mutated.geneticRepresentation.resize(ind.geneticRepresentation.size());
		for (size_t j = 0; j < mutated.geneticRepresentation.size(); j++) {
			mutated.geneticRepresentation[j] = (rand() % 2 == 0 ?
				ind.geneticRepresentation[j] :
				rand() % 2 == 0);
		}
		newPopulation.push_back(mutated);
		mutatedInd++;
		i++;
	}
	population = newPopulation;
	printf("Reproduction: 1 individual from previous population, %d individual from crossovers and %d from mutation.\n",
		crossoverInd, mutatedInd);
}

int main(int argc, const char** argv)
{
	OptionsReader opreader(argc, argv);
	int epochs, geneticLen, populationSize;
	opreader.param("epochs", "Epochs", epochs, 100);
	opreader.param("geneticLen", "Genetic representation length", geneticLen, 50);
	opreader.param("populationSize", "(Constant) population size", populationSize, 100);
	
	ofs.open("fitness");
	KnapsackProblem kp(geneticLen);
	kp.initialize(populationSize);
	for (size_t e = 0; e < (uint) epochs; e++) {
		printf("Current best fitness: %.2f.\n\n", kp.getCurrentBestFitness());
		ofs << kp.getCurrentBestFitness() << endl;
		kp.iteration();
	}
	ofs.close();
	return 0;
}
