/**
\mainpage Petri Net Plans

The Petri Net Plan (PNP) formalism allows for high level description of 
complex action interactions that are necessary in programming cognitive 
agents with real-time requirements, such as mobile robots or video game AIs. 
PNPs are inspired to languages for reasoning about actions, yet are more 
expressive than most of them, offering a full fledged set of operators 
for dealing with   non-instantaneous actions, sensing actions, action 
failures, concurrent actions and cooperation in a multi agent context.

For an introduction to PNP please refer to http://pnp.dis.uniroma1.it

\section who Who to Bother

PNP has been initially developed by Vittorio A. Ziparo ( http://www.dis.uniroma1.it/~ziparo/ )

It has then been improved and maintained for a while by Daniele Calisi ( http://www.dis.uniroma1.it/~calisi/ )

It is currently in the clutches of Matteo Leonetti ( http://www.dis.uniroma1.it/~leonetti/ ) who has
also developed the learning part. Please report to him any mistake you might find in this documentation.

\section usage General Usage

You want to more or less follow this list:

-# Write a plan and save it in the format you prefer. We suggest the Petri 
   Net Markup Language ( http://www.informatik.hu-berlin.de/top/pnml/ ) so that the
   plan loader will come for free (we developed one for pnml, otherwise it's on you). 
   Our favorite tool to do so is Jarp ( http://jarp.sourceforge.net/ )
-# At this point you should have clarified to yourself what actions you need. Implement 
   the actions subclassing for each of them PetriNetPlans::PnpAction and overriding the 
   methods you want (typically among PetriNetPlans::PnpAction::start(), 
   PetriNetPlans::PnpAction::executeStep(), PetriNetPlans::PnpAction::end())
   Pay attention to the termination condition, if you want it to be internal (triggered by the action rather than the plan)
   you must also override PetriNetPlans::PnpAction::finished().
-# Make sure you have the interface between the agent and the environment
   ready. This can be as complicated as a Knowledge Base updated according
   to the agent perceptions or as simple as a synthetic representation
   of the environment itself (e.g. the board in tic-tac-toe).
-# Subclass PetriNetPlans::ExternalConditionChecker and implement PetriNetPlans::ExternalConditionChecker::evaluateAtomicExternalCondition()
   to test the conditions you used in the plan(s) on your interface to the environment.
   This class is the bridge between PNP and the outside world.
-# Subclass PetriNetPlans::ExecutableInstantiator and have it create your plans and actions. PNP doesn't
   distinguish among them so they are all returned as PetriNetPlans::PnpExecutable instances. 
   This object must know where the plans are stored and what parameters (if any) provide
   to the actions. None of these things are enforced by PNP so you can implement this
   class in many different ways and in complicated systems it might require some
   ingenuity. To load plans
   from pnml files you can use PetriNetPlans::XMLPnpPlanInstantiator: create an empty plan and pass it
   to PetriNetPlans::XMLPnpPlanInstantiator::loadFromPNML(). This object also commonly passes itself to the newly created plan
   to be its instantiator too.
-# Finally instantiate PetriNetPlans::PnpExecuter, set the name of the main plan with 
   PetriNetPlans::PnpExecuter::setMainPlan() and repeatedly call PetriNetPlans::PnpExecuter::execMainPlanStep() at the frequency you want
   (this depends on how often you want your agent to make decisions that in turn usually
   depends on how fast the environment changes).
   
This list is in the order of the dependencies but I suggest to write the plan first and then
follow the other steps in reverse order, so that you'll clearly see when and where you
need every component before actually implementing it.

\section learning PNP and Reinforcement Learning

\b disclaimer: RL on PNP is still under development and testing.

To use the learning capability you must follow the same steps as for normal PNP with
a few differences:

- Instead of PetriNetPlans::ExternalConditionChecker you must subclass learnpnp::RewardCollector. It IS-A
  ExternalConditionChecker but in addition has a function to communicate to PNP
  the reward obtained between two subsequent calls of learnpnp::RewardCollector::reward().
  Since both accumulating the reward and testing conditions need access to the agent's
  knowledge about the environment they are implemented by the same component.
- The executable instantiator (the subclass of PetriNetPlans::ExecutableInstantiator) must return
  through the method PetriNetPlans::ExecutableInstantiator::createExecutable() an instance of learnpnp::LearnPlan instead of PetriNetPlans::PnpPlan.

You can choose among the learning algorithms in pnp/learning_plan/algo/ and the exploration strategies
in pnp/learning_plan/exp/ and provide one of each to the constructor of LearnPlan. You could also write
your own subclassing respectively learnpnp::Learner and learnpnp::ExpPolicy.
**/

