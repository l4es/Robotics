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

#ifndef GENETICALGORITHMS_BASEGENETICALGORITHM_H
#define GENETICALGORITHMS_BASEGENETICALGORITHM_H

#include <vector>
#include <float.h>
#include <math.h>

namespace GeneticAlgorithms {

using namespace std;

template<typename Gene, typename UserIndividual = void>
class BaseIndividual {
public:
	vector<Gene> geneticRepresentation;
	UserIndividual* userIndividual;
	
	virtual ~BaseIndividual() { }
};

class BaseFitness {
protected:
	double val;

public:
	BaseFitness(double v) : val(fabs(v)) { }
	virtual ~BaseFitness() { }
	virtual inline operator double() const { return val; }
};

struct FitnessGreaterIsBetter : public BaseFitness {
	FitnessGreaterIsBetter(double v = 0.) : BaseFitness(v) { }
	bool operator<(const FitnessGreaterIsBetter& f) const { return val > f.val; }
	static FitnessGreaterIsBetter worst() { return FitnessGreaterIsBetter(0.); }
	static FitnessGreaterIsBetter best() { return FitnessGreaterIsBetter(DBL_MAX); }
};

struct FitnessLesserIsBetter : public BaseFitness {
	FitnessLesserIsBetter(double v = 0.) : BaseFitness(v) { }
	bool operator<(const FitnessLesserIsBetter& f) const { return val < f.val; }
	static FitnessLesserIsBetter worst() { return FitnessLesserIsBetter(DBL_MAX); }
	static FitnessLesserIsBetter best() { return FitnessLesserIsBetter(0.); }
};

template<typename Individual, typename Fitness = FitnessGreaterIsBetter>
class BaseGeneticAlgorithm {
public:
	vector<Individual> population;

public:
	BaseGeneticAlgorithm() { }
	virtual ~BaseGeneticAlgorithm() { }
	
	virtual void iteration();
	
	virtual double getCurrentBestFitness() {
		Fitness bestFit = Fitness::worst();
		for (size_t i = 0; i < population.size(); i++) {
			Fitness curFit = fitness(population[i]);
			if (curFit < bestFit) bestFit = curFit;
		}
		return bestFit;
	}

	virtual Individual getCurrentBestIndividual() {
		uint bestIdx = 0;
		Fitness bestFit = Fitness::worst();
		for (size_t i = 0; i < population.size(); i++) {
			Fitness curFit = fitness(population[i]);
			if (curFit < bestFit) {
				bestFit = curFit;
				bestIdx = i;
			}
		}
		return population[bestIdx];
	}

protected:
	virtual Fitness fitness(const Individual& individual) = 0;
	virtual vector<Individual> selection() = 0;
	virtual void reproduction(vector<Individual> selectedPopulation) = 0;
};

template<typename Individual, typename Fitness>
void BaseGeneticAlgorithm<Individual, Fitness>::iteration()
{
	vector<Individual> selectedPopulation = selection();
	reproduction(selectedPopulation);
}

}

#endif
