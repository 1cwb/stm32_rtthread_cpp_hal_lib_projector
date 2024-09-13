#include "mobject.hpp"
/**
 * This function will return the specified type of object information.
 *
 * @param type the type of object, which can be
 *             RT_Object_Class_Thread/Semaphore/Mutex... etc
 *
 * @return the object type information or nullptr
 */
mObjectInformation_t *mObject::objectGetInformation(mObjectClassType type)
{
    int index;

    for (index = 0; index < M_OBJECT_INFO_UNKNOWN; index ++)
        if (objectContainer[index].type == type) return &objectContainer[index];

    return nullptr;
}
/**
 * This function will return the length of object list in object container.
 *
 * @param type the type of object, which can be
 *             RT_Object_Class_Thread/Semaphore/Mutex... etc
 * @return the length of object list
 */
int mObject::objectGetLength(mObjectClassType type)
{
    int count = 0;
    unsigned long level;
    struct mList_t *node = nullptr;
    struct mObjectInformation_t *information = nullptr;

    information = objectGetInformation(type);
    if (information == nullptr) return 0;

    level = HW::hwInterruptDisable();
    /* get the count of objects */

    for(node = information->objectList.next; node != &(information->objectList); node = node->next)
    {
        count ++;
    }
    HW::hwInterruptEnable(level);

    return count;
}
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
int mObject::objectGetPointers(mObjectClassType type, mObject_t **pointers, int maxlen)
{
    int index = 0;
    unsigned long level;

    struct mObject_t *object;
    struct mList_t *node = nullptr;
    struct mObjectInformation_t *information = nullptr;

    if (maxlen <= 0) return 0;

    information = objectGetInformation(type);
    if (information == nullptr) return 0;

    level = HW::hwInterruptDisable();
    /* retrieve pointer of object */
    for(node = information->objectList.next; node != &(information->objectList); node = node->next)
    {
        object = listEntry(node, mObject_t, list);

        pointers[index] = object;
        index ++;

        if (index >= maxlen) break;
    }
    HW::hwInterruptEnable(level);

    return index;
}
/**
 * This function will initialize an object and add it to object system
 * management.
 *
 * @param object the specified object to be initialized.
 * @param type the object type.
 * @param name the object name. In system, the object's name must be unique.
 */
void mObject::objectInit(mObject_t *object, mObjectClassType type, const char *name)
{
    register long temp;
    struct mList_t *node = nullptr;
    struct mObjectInformation_t *information;

    /* get object information */
    information = objectGetInformation(type);
    MASSERT(information != nullptr);

    /* check object type to avoid re-initialization */

    /* enter critical */

    mSchedule::getInstance()->enterCritical();
    /* try to find object */
    for (node = information->objectList.next; node != &(information->objectList); node = node->next)
    {
        mObject_t *obj;

        obj = listEntry(node, mObject_t, list);
        if (obj) /* skip warning when disable debug */
        {
            MASSERT(obj != object);
        }
    }

    /* leave critical */
    mSchedule::getInstance()->exitCritical();

    /* initialize object's parameters */
    /* set object type to static */
    object->type = type | M_OBJECT_CLASS_STATIC;

    /* copy name */
    strncpy(object->name, name, NAME_MAX);

    //RT_OBJECT_HOOK_CALL(rt_object_attach_hook, (object));

    /* lock interrupt */
    temp = HW::hwInterruptDisable();

    /* insert object into information object list */
    object->list.insertAfterTo(&(information->objectList));

    /* unlock interrupt */
    HW::hwInterruptEnable(temp);
}
/**
 * This function will detach a static object from object system,
 * and the memory of static object is not freed.
 *
 * @param object the specified object to be detached.
 */
void mObject::objectDetach(mObject_t* object)
{
    register long temp;

    /* object check */
    MASSERT(object != nullptr);

    //RT_OBJECT_HOOK_CALL(rt_object_detach_hook, (object));

    /* reset object type */
    object->type = 0;

    /* lock interrupt */
    temp = HW::hwInterruptDisable();

    /* remove from old list */
    object->list.removeSelf();

    /* unlock interrupt */
    HW::hwInterruptEnable(temp);
}
/**
 * This function will allocate an object from object system
 *
 * @param type the type of object
 * @param name the object name. In system, the object's name must be unique.
 *
 * @return object
 */
mResult mObject::objectAdd(struct mObject_t *object, const mObjectClassType type, const char *name)
{
    register long temp;
    struct mObjectInformation_t *information;

    //RT_DEBUG_NOT_IN_INTERRUPT;
    if (object == nullptr)
    {
        /* no memory can be allocated */
        return M_RESULT_ERROR;
    }
    /* get object information */
    information = objectGetInformation(type);
    MASSERT(information != nullptr);

    /* clean memory data of object */
    //memset(object, 0x0, information->objectSize);

    /* initialize object's parameters */

    /* set object type */
    object->type = type;

    /* set object flag */
    object->flag = 0;

    /* copy name */
    strncpy(object->name, name, NAME_MAX);

    /* lock interrupt */
    temp = HW::hwInterruptDisable();

    /* insert object into information object list */
    object->list.insertAfterTo(&(information->objectList));

    /* unlock interrupt */
    HW::hwInterruptEnable(temp);

    /* return object */
    return M_RESULT_EOK;
}

/**
 * This function will delete an object and release object memory.
 *
 * @param object the specified object to be deleted.
 */
void mObject::objectRemove(struct mObject_t* object)
{
    register long temp;

    /* object check */
    MASSERT(object != nullptr);
    MASSERT(!(object->type & M_OBJECT_CLASS_STATIC));

    /* reset object type */
    object->type = M_OBJECT_CLASS_NULL;

    /* lock interrupt */
    temp = HW::hwInterruptDisable();

    /* remove from old list */
    object->list.removeSelf();

    /* unlock interrupt */
    HW::hwInterruptEnable(temp);
}
/**
 * This function will return the type of object without
 * RT_Object_Class_Static flag.
 *
 * @param object the specified object to be get type.
 *
 * @return the type of object.
 */
uint8_t mObject::objectGetType(mObject_t* object)
{
    /* object check */
    MASSERT(object != nullptr);

    return object->type & ~M_OBJECT_CLASS_STATIC;
}
/**
 * This function will judge the object is system object or not.
 * Normally, the system object is a static object and the type
 * of object set to RT_Object_Class_Static.
 *
 * @param object the specified object to be judged.
 *
 * @return RT_TRUE if a system object, RT_FALSE for others.
 */
bool mObject::objectIsSystemobject(mObject_t* object)
{
    /* object check */
    MASSERT(object != nullptr);

    if (object->type & M_OBJECT_CLASS_STATIC)
        return true;

    return false;
}
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
mObject_t* mObject::objectFind(const char *name, mObjectClassType type)
{
    struct mObject_t *object = nullptr;
    struct mList_t *node = nullptr;
    struct mObjectInformation_t *information = nullptr;

    information = mObject::objectGetInformation(type);

    /* parameter check */
    if ((name == nullptr) || (information == nullptr)) return nullptr;

    /* which is invoke in interrupt status */
    //DEBUG_NOT_IN_INTERRUPT;

    /* enter critical */
    mSchedule::getInstance()->enterCritical();

    /* try to find object */
    for(node = information->objectList.next; node != &(information->objectList); node = node->next)
    {
        object = listEntry(node, mObject_t, list);
        if (strncmp(object->name, name, NAME_MAX) == 0)
        {
            /* leave critical */
            mSchedule::getInstance()->exitCritical();

            return object;
        }
    }

    /* leave critical */
    mSchedule::getInstance()->exitCritical();

    return nullptr;
}

void mObject::objectInfoInit()
{
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_THREAD, M_OBJECT_CLASS_THREAD);
/* initialize object container - semaphore */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_SEMAPHORE, M_OBJECT_CLASS_SEMAPHORE);
/* initialize object container - mutex */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_MUTEX, M_OBJECT_CLASS_MUTEX);
/* initialize object container - event */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_EVENT, M_OBJECT_CLASS_EVENT);
#ifdef M_USING_MAILBOX
/* initialize object container - mailbox */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_MAILBOX, M_OBJECT_CLASS_MAILBOX);
#endif
/* initialize object container - message queue */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_MESSAGEQUEUE, M_OBJECT_CLASS_MESSAGEQUEUE);
#ifdef M_USING_MEMHEAP
/* initialize object container - memory heap */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_MEMHEAP, M_OBJECT_CLASS_MEMHEAP);
#endif
#ifdef M_USING_MEMPOOL
/* initialize object container - memory pool */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_MEMPOOL, M_OBJECT_CLASS_MEMPOLL);
#endif
#ifdef M_USING_DEVICE
/* initialize object container - device */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_DEVICE, M_OBJECT_CLASS_DEVICE);
#endif
/* initialize object container - timer */
    _OBJ_CONTAINER_INIT(M_OBJECT_INFO_TIMER, M_OBJECT_CLASS_TIMER);
}