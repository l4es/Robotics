
ADD_LIBRARY(searchalgorithms SHARED basesearchalgorithm.cpp)

IF(ENABLE_SEARCHALGORITHMS_TESTS)
	RDK_ADD_TEST(test_searchingrid_dijkstra)
	RDK_ADD_TEST(test_searchingrid_astar)
	RDK_ADD_TEST(test_searchingrid_astar2)
	RDK_ADD_TEST(test_searchingrid_bestfirst)
ENDIF(ENABLE_SEARCHALGORITHMS_TESTS)
