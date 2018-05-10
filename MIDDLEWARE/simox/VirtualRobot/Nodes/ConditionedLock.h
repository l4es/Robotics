#ifndef __CONDITIONED_LOCK__
#define __CONDITIONED_LOCK__



template <class T>
class ConditionedLock
{
private:
    T _lock;
    bool _enabled;
public:
    ConditionedLock(boost::recursive_mutex&   mutex, bool enabled) :
        _lock(mutex, boost::defer_lock), _enabled(enabled)
    {
        if (_enabled)
        {
            _lock.lock();
        }
    }
    ~ConditionedLock()
    {
        if (_enabled)
        {
            _lock.unlock();
        }
    }
};

typedef ConditionedLock<boost::unique_lock<boost::recursive_mutex> > ReadLock;
typedef ConditionedLock<boost::unique_lock<boost::recursive_mutex> > WriteLock;
//typedef ConditionedLock<boost::shared_lock<boost::shared_mutex> > ReadLock;
//typedef ConditionedLock<boost::unique_lock<boost::shared_mutex> > WriteLock;
typedef boost::shared_ptr< ReadLock > ReadLockPtr;
typedef boost::shared_ptr< WriteLock > WriteLockPtr;

#endif
