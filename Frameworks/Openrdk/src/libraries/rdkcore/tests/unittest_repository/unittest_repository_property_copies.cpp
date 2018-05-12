#include <cstdio>
#include <iostream>
using namespace std;

#include <rdkcore/logging/logging.h>
#include <rdkcore/repository/repository.h>
#include <rdkcore/repository/session.h>
#include <rdkcore/rgeometry/rpoint2i.h>
#include <rdkcore/ns.h>
using namespace RDK2;

#define LOGGING_MODULE "TestRepository"

int r = 0;

struct TestObject {
	double d;
	int i;
	TestObject() { printf("TestObject::__default_constructor\n"); r += 1; }
	~TestObject() { printf("TestObject::__default_destructor\n"); r += 2; }
	TestObject(const TestObject& t) { printf("TestObject::__copy_constructor\n"); d = t.d; i = t.i; r += 4; }
	TestObject& operator=(const TestObject& t) { printf("TestObject::operator=\n"); d = t.d; i = t.i; r += 8; return *this; }
};

struct RTestObject : public RDK2::Object, public TestObject {
	RTestObject() : RDK2::Object(), TestObject() { printf("RTestObject::__default_constructor\n"); r += 16; }
	~RTestObject() { printf("RTestObject::__default_constructor\n"); r += 32; }
	RTestObject(const RTestObject& t) : TestObject(t) { printf("RTestObject::__copy_constructor\n"); r += 64; }
	RTestObject& operator=(const RTestObject& t) { TestObject::operator=(t); printf("RTestObject::operator=\n"); r += 128; return *this; }
};

int main(int, char**)
{
	bool ok = true;

	Repository* repository = new Repository("testRepository");
	Session* session = repository->createSession("testSession", "Test session", "/");

	SESSION_TRY_START(session);

	session->createDouble("/myDouble", "My double", RDouble::M, 42.0);
	session->createStorage("RTestObject", "/myTestObject", "My test object");
	session->setObject("/myTestObject", new RTestObject());

	printf("\n\nStarting tests\n");

	r = 0;
	printf("\nBEGIN RTestObject rto = session->getObjectCopyAs<RTestObject>(...)\n");
	RTestObject rto = session->getObjectCopyAs<RTestObject>("/myTestObject");
	printf("END (r = %d, expected %d)\n", r, 68);
	if (r != 68) ok = false;

	r = 0;
	printf("\nBEGIN TestObject to = session->getObjectCopyAs<RTestObject>(...)\n");
	TestObject to = session->getObjectCopyAs<RTestObject>("/myTestObject");
	printf("END (r = %d, expected %d)\n", r, 4);
	if (r != 4) ok = false;
	
	r = 0;
	printf("\nBEGIN rto = session->getObjectCopyAs<RTestObject>(...)\n");
	rto = session->getObjectCopyAs<RTestObject>("/myTestObject");
	printf("END (r = %d, expected %d)\n", r, 136);
	if (r != 136) ok = false;

	r = 0;
	printf("\nBEGIN to = session->getObjectCopyAs<RTestObject>(...)\n");
	to = session->getObjectCopyAs<RTestObject>("/myTestObject");
	printf("END (r = %d, expected %d)\n", r, 8);
	if (r != 8) ok = false;

	r = 0;
	printf("\nBEGIN RTestObject rto2;\n");
	RTestObject rto2;
	printf("END (r = %d, expected %d)\n", r, 17);
	if (r != 17) ok = false;
	r = 0;
	printf("BEGIN session->copyObjectAs<RTestObject>(..., rto2);\n");
	session->copyObject<RTestObject>("/myTestObject", rto2);
	printf("END (r = %d, expected %d)\n", r, 136);
	if (r != 136) ok = false;

	// NOTE: this is currently not possible
	// printf("\nBEGIN TestObject to2;\n");
	// TestObject to2;
	// printf("END\n");
	// printf("BEGIN session->copyObjectAs<TestObject>(..., to2);\n");
	// session->copyObject<TestObject>("/myTestObject", to2);
	// printf("END\n");
	
	double d = session->getObjectCopyAs<RDouble>("/myDouble");
	printf("\nGot %f from double (expected %f)\n", d, 42.0);
	if (d != 42.0) ok = false;
	
	printf("\n\n");
	
	SESSION_END_CATCH_TERMINATE(session);

	
	delete session;
	delete repository;	

	if (!ok) printf("SOMETHING WRONG... please check the code of %s and the meaning of the numbers\n");

	return (ok ? 0 : 1);
}
