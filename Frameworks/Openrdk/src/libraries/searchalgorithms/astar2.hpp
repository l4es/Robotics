namespace SearchAlgorithms {

template<typename UserStateClass>
list<UserStateClass> AStar2<UserStateClass>::search(const UserStateClass& userStartState, const UserStateClass& userTargetState)
{
	typedef LabeledState<UserStateClass> LState;
	LabeledState<UserStateClass>* labeledStartState = new LabeledState<UserStateClass>(userStartState);
	labeledStartState->costToCome = 0;
	labeledStartState->estimatedCostToGo = heuristicFunction(userStartState, userTargetState);
	labeledStartState->parent = 0;
	uniqueSearchedStatesInLastSearch = 0;
	searchedStatesInLastSearch = 0;
	this->aliveLStates.push(labeledStartState);
	this->visitedLStates.push_back(labeledStartState);	// this is for computing the path
	while (!aliveLStates.empty()) {
		// pop the best state
		LabeledState<UserStateClass>* curLState = aliveLStates.top();
		aliveLStates.pop();

		// if it is already been visited, skip it
		if (isExpanded(curLState->userState)) continue;
		
		// if it is the target state, we've done
		if (isTargetState(curLState->userState, userTargetState)) {
			list<UserStateClass> plan;
			for (LabeledState<UserStateClass>* ls = curLState; ls != 0; ls = ls->parent) {
				plan.push_back(ls->userState);
			}
			for (typename vector<LState*>::iterator it = visitedLStates.begin(); it != visitedLStates.end(); ++it) {
				delete *it;
			}
			return plan.reverse(), plan;
		}

		// take the successors of the current state
		vector<UserStateClass> successors = getSuccessorsOf(curLState->userState);
		debugSuccessorStates(successors);

		// mark the current state as visited
		markExpanded(curLState->userState);

		// for each successor...
		for (typename vector<UserStateClass>::iterator it = successors.begin(); it != successors.end(); ++it) {
			if (isExpanded(*it)) {
				continue;
			}
			else if (!isEnqueued(*it)) {
				uniqueSearchedStatesInLastSearch++;
				searchedStatesInLastSearch++;
				LabeledState<UserStateClass>* curLSucc = new LabeledState<UserStateClass>(*it);
				// compute its cost
				curLSucc->costToCome = curLState->costToCome + computeCost(curLState->userState, curLSucc->userState);
				curLSucc->estimatedCostToGo = heuristicFunction(curLState->userState, userTargetState);
				// put the parent
				curLSucc->parent = curLState;
				// put in the queue
				aliveLStates.push(curLSucc);
				visitedLStates.push_back(curLSucc);
				markEnqueued(curLSucc->userState);
			}
			else {
				// if it is already been enqueued, it has already the cost computed, so maybe we have to update it
				searchedStatesInLastSearch++;
				for (size_t i = 0; i < visitedLStates.size(); i++) {
					LabeledState<UserStateClass>* aLState = visitedLStates[i];
					if (*it == visitedLStates[i]->userState) {
						double newCostToCome = curLState->costToCome + computeCost(curLState->userState, aLState->userState);
						if (newCostToCome < aLState->costToCome) {
							aLState->costToCome = newCostToCome;
							aLState->parent = curLState;
						}
						//aliveLStates.push(aLState);
						break;
					}
				}
			}
		}
	}
	return list<UserStateClass>();
}

}
