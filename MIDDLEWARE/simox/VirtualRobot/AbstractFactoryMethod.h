// *****************************************************************
// Declaration of AbstractFactoryMethod
// *****************************************************************
// Author:  Nils Adermann
// Date:    03.08.2009
// *****************************************************************

#ifndef ABSTRACT_FACTORY_METHOD_H
#define ABSTRACT_FACTORY_METHOD_H

#include <vector>
#include <string>
#include <map>


/**
* A template that can be used as a superclass of a class hierarchy that
* wants to provide a factory method which allows instantiation of objects
* based on a string identifier.
*
* The first template argument is the base class of your class hierarchy.
* The second argument is the parameter type for the initialisation function
* each subclass has to provide. If you need multiple constructor arguments
* it is recommended to use a boost::tuple.
*/
template <typename Base, typename constructorArg>
class AbstractFactoryMethod
{
public:
    /**
    * The function pointer type of subclass initialisation functions.
    * This matches the createInstance method.
    */
    typedef boost::shared_ptr<Base> (*initialisationFunction)(constructorArg);

    /**
    * Function which can be used to retrieve an object specified by string name.
    */
    static boost::shared_ptr<Base> fromName(const std::string& name, constructorArg params)
    {
        if (subTypes()->find(name) == subTypes()->end())
        {
            return boost::shared_ptr<Base>();
        }

        boost::shared_ptr<Base> instance = (*subTypes())[name](params);
        instance->setDescription(name);
        return instance;
    }

    /**
    * Function which can be used to retrieve the first registered object.
    */
    static boost::shared_ptr<Base> first(constructorArg params)
    {
        if (subTypes()->size() == 0)
        {
            return boost::shared_ptr<Base>();
        }

        boost::shared_ptr<Base> instance = (*subTypes()).begin()->second(params);
        instance->setDescription((*subTypes()).begin()->first);
        return instance;
    }

    /**
    * Returns the class's name. This is used to identify the class which the
    * user requests an instance of.
    */
    static std::string getName()
    {
        return "AbstractFactoryMethod";
    }

    /**
    * Initialisation function which needs to be provided by every subclass.
    * It calls the constructor and returns a shared_ptr to the resulting
    * object.
    */
    static boost::shared_ptr<Base> createInstance(constructorArg)
    {
        return boost::shared_ptr<Base>();
    }

    /**
    * Statically called by subclasses to register their name and initialisation
    * function so they can be found by {@link fromName fromName}.
    */
    static void registerClass(const std::string& name, initialisationFunction init)
    {
        (*subTypes())[name] = init;
    }

    /**
     * Set a description on the instance it is called on.
     */
    void setDescription(const std::string& newDescription)
    {
        description = newDescription;
    }

    /**
     * Return the description of the current instance.
     */
    std::string getDescription() const
    {
        return description;
    }

    /**
     * Return a list of all registered subclasses.
     */
    static std::vector<std::string> getSubclassList()
    {
        std::vector<std::string> subclassList;
        typename std::map<std::string, initialisationFunction>::const_iterator iter = subTypes()->begin();

        for (; iter != subTypes()->end(); ++iter)
        {
            subclassList.push_back(iter->first);
        }

        return subclassList;
    }

    /**
    * A helper struct to allow static initialisation of the subclass lookup table.
    */
    struct SubClassRegistry
    {
        SubClassRegistry(const std::string& name, initialisationFunction init)
        {
            Base::registerClass(name, init);
        }
    };

private:
    /**
    * Static wrapper method for accessing subTypes map.
    * This method is necessary to make certain that the map is initialised
    * before use. This can only be guaranteed through a static local variable
    * in a function.
    */
    static boost::shared_ptr<std::map<std::string, initialisationFunction> > subTypes()
    {
        static boost::shared_ptr<std::map<std::string, initialisationFunction> > subTypes(new std::map<std::string, initialisationFunction>);

        return subTypes;
    }

    std::string description;
};

#endif /* ABSTRACT_FACTORY_METHOD_H */
