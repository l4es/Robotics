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
#include <cmath>
#include <map>
using namespace std;

#include <optionspp/optionsreader.h>
using namespace Options;

#include "basegeneticalgorithm.h"
using namespace GeneticAlgorithms;

ofstream ofs;

typedef GeneticAlgorithms::BaseIndividual<double> EquationIndividual;

class EquationProblemGaSolver : public BaseGeneticAlgorithm<EquationIndividual, FitnessLesserIsBetter> {
public:
	void initialize(size_t populationSize);

protected:
	FitnessLesserIsBetter fitness(const EquationIndividual& individual);
	vector<EquationIndividual> selection();
	void reproduction(vector<EquationIndividual> selectedPopulation);
};

void EquationProblemGaSolver::initialize(size_t populationSize)
{
	population.clear();
	for (size_t i = 0; i < populationSize; i++) {
		EquationIndividual ind;
		ind.geneticRepresentation.resize(5);
		for (size_t j = 0; j < ind.geneticRepresentation.size(); j++) {
			ind.geneticRepresentation[j] = (rand() % 2000 - 1000) / 100.;
		}
		population.push_back(ind);
	}
}

FitnessLesserIsBetter EquationProblemGaSolver::fitness(const EquationIndividual& individual)
{
	const double& x = individual.geneticRepresentation[0];
	const double& y = individual.geneticRepresentation[1];
	const double& z = individual.geneticRepresentation[2];
	const double& t = individual.geneticRepresentation[3];
	const double& w = individual.geneticRepresentation[4];
	return x * x + 4 * y + t * w + w * w - 4 * x * y * z;
}

vector<EquationIndividual> EquationProblemGaSolver::selection()
{
	printf("Current population: %d individuals.\n", population.size());
	multimap<FitnessLesserIsBetter, EquationIndividual*> m;
	for (size_t i = 0; i < population.size(); i++) {
		m.insert(make_pair(fitness(population[i]), &population[i]));
	}
	vector<EquationIndividual> ret;
	size_t i = 0, max = population.size() / 2;
	for (multimap<FitnessLesserIsBetter, EquationIndividual*>::iterator it = m.begin(); it != m.end(); ++it) {
		ret.push_back(*(it->second));
		if (i++ > max) break;
	}
	printf("Selected %d individual for reproduction.\n", ret.size());
	return ret;
}

void EquationProblemGaSolver::reproduction(vector<EquationIndividual> selectedPopulation)
{
	vector<EquationIndividual> newPopulation;
	newPopulation.push_back(selectedPopulation[0]);	// the best individual of previous population
	// crossover
	size_t crossoverInd = 0;
	for (size_t i = 0; i < selectedPopulation.size()/2; i++) {
		EquationIndividual& ind1 = selectedPopulation[i];
		EquationIndividual& ind2 = selectedPopulation[i+1];
		EquationIndividual child;
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
		EquationIndividual& ind = (i < selectedPopulation.size() ? selectedPopulation[i] : selectedPopulation[0]);
		EquationIndividual mutated;
		mutated.geneticRepresentation.resize(ind.geneticRepresentation.size());
		for (size_t j = 0; j < mutated.geneticRepresentation.size(); j++) {
			switch (rand() % 3) {
				case 0: mutated.geneticRepresentation[j] = ind.geneticRepresentation[j]; break;
				case 1: mutated.geneticRepresentation[j] = (rand() % 2000 - 1000) / 100.; break;
				case 2: mutated.geneticRepresentation[j] = 
					ind.geneticRepresentation[j] + (rand() % 2000 - 1000) / 1000.; break;
			}
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
	int epochs, populationSize;
	opreader.param("epochs", "Epochs", epochs, 100);
	opreader.param("populationSize", "(Constant) population size", populationSize, 100);
	
	ofs.open("fitness");
	EquationProblemGaSolver ep;
	ep.initialize(populationSize);
	for (size_t e = 0; e < (uint) epochs; e++) {
		printf("Current best fitness: %.2f.\n\n", ep.getCurrentBestFitness());
		ofs << ep.getCurrentBestFitness() << endl;
		ep.iteration();
	}
	ofs.close();
	EquationIndividual bestIndividual = ep.getCurrentBestIndividual();
	printf ("Found solution x = %.4f, y = %.4f, z = %.4f, t = %.4f, w = %.4f.\n",
		bestIndividual.geneticRepresentation[0],
		bestIndividual.geneticRepresentation[1],
		bestIndividual.geneticRepresentation[2],
		bestIndividual.geneticRepresentation[3],
		bestIndividual.geneticRepresentation[4]);
		
	return 0;
}
