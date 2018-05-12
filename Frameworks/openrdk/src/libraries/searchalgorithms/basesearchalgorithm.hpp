namespace SearchAlgorithms {

template<typename UserStateClass, typename CostClass>
list<UserStateClass> BaseSearchAlgorithm<UserStateClass, CostClass>::search(const UserStateClass& uStateStart,
	const UserStateClass& uTargetState)
{
	userTargetState = uTargetState;
	return search(uStateStart);
}

template<typename UserStateClass, typename CostClass>
bool BaseSearchAlgorithm<UserStateClass, CostClass>::isTargetState(const UserStateClass& userState)
{
	return isSameState(userState, userTargetState);
}


template<typename UserStateClass, typename CostClass>
bool BaseSearchAlgorithm<UserStateClass, CostClass>::
	isDominated(const UserStateClass& s, CostClass costToCome)
{
	typename list<pair<UserStateClass, State*> >::iterator itF = visitedStates.begin();
	for ( ; itF != visitedStates.end(); ++itF)
		if ( (equal(itF->second->costToCome, costToCome)
			|| lessThan(itF->second->costToCome, costToCome))
				&& isDominatedBy(s, itF->first)) { 
			//std::cout << s << " is dominated by " << itF->first << endl;
			return true;	
		}
	
	return false;
}

template<typename UserStateClass, typename CostClass>
void BaseSearchAlgorithm<UserStateClass, CostClass>::
	eraseAliveDominatedBy(const UserStateClass& s, CostClass costToCome) {
				
	for (typename list<State*>::iterator itA = aliveStates.begin();
		itA != aliveStates.end(); ) {
		if ((equal(costToCome, (*itA)->costToCome) ||
				lessThan(costToCome, (*itA)->costToCome))
				&& isDominatedBy((*itA)->userState, s)) 
			itA = aliveStates.erase(itA);
		else 
			++itA;
	}
}

template<typename UserStateClass, typename CostClass>
list<UserStateClass> BaseSearchAlgorithm<UserStateClass, CostClass>::search(const UserStateClass& uStateStart)
{
	searchedStates = uniqueSearchedStates = 0;
	State* stateStart = new State(uStateStart);
	stateStart->costToCome = 0; // = 20-50euro
	visitedStates.push_back(make_pair(uStateStart, stateStart));
	aliveStates.push_back(stateStart);
	int num = 0;
	while (!aliveStates.empty() && num++<30000) {
		/*
		for (typename list<State*>::iterator itA = aliveStates.begin();
			itA != aliveStates.end(); itA++) {
				cout << "Num " << num << " has " << (*itA)->userState << " cost " 
				<< (*itA)->costToCome << endl;
		}*/
		
		State* currentState = popFirstAliveState();
		if (!currentState) {
			throw std::runtime_error("popFirstAliveState() returned 0.");
		}
		
		if (isTargetState(currentState->userState)) {
			return makePlanAndFreeStates(currentState);
		}
		vector<UserStateClass> nextUserStates = getSuccessorsOf(currentState->userState);
		/*cout << "==  " << num << " Expansion of " << currentState->userState << " gave " 
		   << nextUserStates.size() << " children." << endl;*/
		int i=0;
		for (typename vector<UserStateClass>::iterator it = nextUserStates.begin(); it != nextUserStates.end(); ++it) {
			i++;
			
			CostClass newCostToCome = currentState->costToCome +
					computeCost(currentState->userState, *it);
			//cout << i << " = " << *it << endl;
			
			if(isDominated(*it, newCostToCome)) {
				/*cout <<"\t"<< i << " is dominated." << endl;*/
				continue;
			}		
			
			typename list<pair<UserStateClass, State*> >::iterator itF = visitedStates.begin();
			for ( ; itF != visitedStates.end(); ++itF) {
				if (isSameState(*it, itF->first)) {
					break;
				}
			}
			
			//visitedStates.find(*it);
			if (itF == visitedStates.end()) {
				/*cout <<"\t"<< i << "is NEW " << endl;*/
				// the state has not been visited yet
				uniqueSearchedStates++;
				searchedStates++;
				
				State* ns = new State(*it);
				ns->parent = currentState;
				ns->costToCome = sumCosts(currentState->costToCome,
					computeCost(currentState->userState, ns->userState));
				
				visitedStates.push_back(make_pair(*it, ns));
	
				eraseAliveDominatedBy(*it, ns->costToCome);
				
				aliveStates.push_back(ns);
			} else {
				//cout <<"\t"<< i << "is already visited " << endl;
				// the state has already been visited
				searchedStates++;
				//if (costIsBetter(newCostToCome, itF->second->costToCome)) {
				
				if ( lessThan(newCostToCome, itF->second->costToCome) 
					|| (equal(newCostToCome, itF->second->costToCome)
						&& isDominatedBy(itF->second->userState, *it))
				) {
					itF->second->costToCome = newCostToCome;
					itF->second->parent = currentState;
			
					//cout << "found equal state: " << *it << " == " << itF->first << endl;
					//cout << "  old cost: " << itF->second->costToCome << endl;
					//cout << "  new cost: " << newCostToCome << endl;
					
					/*cout <<"\t"<< i << " and cost is better " << newCostToCome << "<" 
						<<itF->second->costToCome << endl;*/
					
					eraseAliveDominatedBy(*it, itF->second->costToCome);
					// Daniele questo ce lo siamo sempre scordato!
					aliveStates.push_back(itF->second);
				} else {
					/*cout <<"\t"<< i << " and cost was less or equal" << newCostToCome << ">" 
						<<itF->second->costToCome << endl;*/
				}
				
			}

			
		}
	}
	freeStates();
	return list<UserStateClass>();
}

template<typename UserStateClass, typename CostClass>
list<UserStateClass> BaseSearchAlgorithm<UserStateClass, CostClass>::makePlanAndFreeStates(State* targetState)
{
	list<UserStateClass> plan;
	State* curState = targetState;
	do {
		plan.push_back(curState->userState);
		curState = curState->parent;
	} while (curState);
	freeStates();
	return plan.reverse(), plan;
}

template<typename UserStateClass, typename CostClass>
void BaseSearchAlgorithm<UserStateClass, CostClass>::freeStates()
{
	for (typename list<pair<UserStateClass, State*> >::iterator it = visitedStates.begin();
	it != visitedStates.end(); ++it) delete it->second;
	visitedStates.clear();
}

} // namespace SearchAlgorithms
