#pragma once
#include "rtoscommon.hpp"
#include "mscheduler.hpp"
#include "mhw.hpp"
#include "mipc.hpp"


enum mObjectInfoType
{
    M_OBJECT_INFO_THREAD = 0,                         /**< The object is a thread. */
    M_OBJECT_INFO_SEMAPHORE,                          /**< The object is a semaphore. */
    M_OBJECT_INFO_MUTEX,                              /**< The object is a mutex. */
    M_OBJECT_INFO_EVENT,                              /**< The object is a event. */
#ifdef M_USING_MAILBOX
    M_OBJECT_INFO_MAILBOX,                            /**< The object is a mail box. */
#endif
    M_OBJECT_INFO_MESSAGEQUEUE,                       /**< The object is a message queue. */
#ifdef M_USING_MEMHEAP
    M_OBJECT_INFO_MEMHEAP,                            /**< The object is a memory heap */
#endif
#ifdef M_USING_MEMPOOL
    M_OBJECT_INFO_MEMPOOL,                            /**< The object is a memory pool. */
#endif
#ifdef M_USING_DEVICE
    M_OBJECT_INFO_DEVICE,                             /**< The object is a device */
#endif
    M_OBJECT_INFO_TIMER,                              /**< The object is a timer. */
    M_OBJECT_INFO_UNKNOWN,                            /**< The object is unknown. */
};

#define _OBJ_CONTAINER_INIT(infoType, classType)     \
    {\
        objectContainer[infoType].type = classType;\
    }

class mObject
{
public:
    static mObject* getInstance()
    {
        static mObject obj;
        return &obj;
    }

    /**
     * This function will return the specified type of object information.
     *
     * @param type the type of object, which can be
     *             RT_Object_Class_Thread/Semaphore/Mutex... etc
     *
     * @return the object type information or nullptr
     */
    mObjectInformation_t *objectGetInformation(mObjectClassType type);
    /**
     * This function will return the length of object list in object container.
     *
     * @param type the type of object, which can be
     *             RT_Object_Class_Thread/Semaphore/Mutex... etc
     * @return the length of object list
     */
    int objectGetLength(mObjectClassType type);
    /**
     * This function will copy the object pointer of the specified type,
     * with the maximum size specified by maxlen.
     *
     * @param type the type of object, which can be
     *             RT_Object_Class_Thread/Semaphore/Mutex... etc
     * @param pointers the pointers will be saved to
     * @param maxlen the maximum number of pointers can be saved
     *
     * @return the copied number of object pointers
     */
    int objectGetPointers(mObjectClassType type, mObject_t **pointers, int maxlen);
    /**
     * This function will initialize an object and add it to object system
     * management.
     *
     * @param object the specified object to be initialized.
     * @param type the object type.
     * @param name the object name. In system, the object's name must be unique.
     */
    void objectInit(mObject_t *object, mObjectClassType type, const char *name);
    /**
     * This function will detach a static object from object system,
     * and the memory of static object is not freed.
     *
     * @param object the specified object to be detached.
     */
    void objectDetach(mObject_t* object);
    /**
     * This function will allocate an object from object system
     *
     * @param type the type of object
     * @param name the object name. In system, the object's name must be unique.
     *
     * @return object
     */
    mResult objectAdd(struct mObject_t *object, const mObjectClassType type, const char *name);

    /**
     * This function will delete an object and release object memory.
     *
     * @param object the specified object to be deleted.
     */
    void objectRemove(struct mObject_t* object);
    /**
     * This function will return the type of object without
     * RT_Object_Class_Static flag.
     *
     * @param object the specified object to be get type.
     *
     * @return the type of object.
     */
    uint8_t objectGetType(mObject_t* object);
    /**
     * This function will judge the object is system object or not.
     * Normally, the system object is a static object and the type
     * of object set to RT_Object_Class_Static.
     *
     * @param object the specified object to be judged.
     *
     * @return RT_TRUE if a system object, RT_FALSE for others.
     */
    bool objectIsSystemobject(mObject_t* object);
    /**
     * This function will find specified name object from object
     * container.
     *
     * @param name the specified name of object.
     * @param type the type of object
     *
     * @return the found object or RT_NULL if there is no this object
     * in object container.
     *
     * @note this function shall not be invoked in interrupt status.
     */
    mObject_t* objectFind(const char *name, mObjectClassType type);

    /**@}*/

private:
    void objectInfoInit();
private:

    mObject()
    {
        objectInfoInit();
    }
    ~mObject()
    {

    }
    mObject(const mObject&) = delete;
    mObject(mObject&&) = delete;
    mObject& operator=(const mObject&) = delete;
    mObject& operator=(mObject&&) = delete;

    mObjectInformation_t objectContainer[M_OBJECT_INFO_UNKNOWN];
};