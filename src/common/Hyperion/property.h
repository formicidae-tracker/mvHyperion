#ifndef propertyH
#define propertyH propertyH


#define _SCAN_PROPERTY_LIST(list,elem,prop_count,prop_id)\
    {\
        int pr, found = 0;\
        elem = list;\
        for( pr = 0; pr < prop_count; pr++ )\
        {\
            if( elem->PropertyID == prop_id )\
            {\
                found = 1;\
                break;\
            }\
            elem++;\
        }\
        if( found == 0 )\
            elem = NULL;\
    }

#define _READ_PROPERTYI(list,prop_count,prop_id,value,changed)\
    {\
        TPropertyElement *elem;\
        _SCAN_PROPERTY_LIST( list, elem, prop_count, prop_id );\
        if( elem != 0 )\
        {\
            value = elem->u.intElement;\
            changed |= elem->Changed;\
        }\
    }

#define _READ_PROPERTYI32(list,prop_count,prop_id,value,changed)\
    {\
        TPropertyElement32 *elem;\
        _SCAN_PROPERTY_LIST( list, elem, prop_count, prop_id );\
        if( elem != 0 )\
        {\
            value = elem->u.intElement;\
            changed |= elem->Changed;\
        }\
    }

#define _WRITE_PROPERTYI(list,prop_id,value,changed)\
    {\
        list->PropertyID = prop_id;\
        list->u.intElement = value;\
        list->Changed = changed;\
        list->Type = prtInt;\
        list++;\
    }

#define _READ_PROPERTYP(list,prop_count,prop_id,pointer,changed)\
    {\
        TPropertyElement *elem;\
        _SCAN_PROPERTY_LIST( list, elem, prop_count, prop_id );\
        if( elem != 0 )\
        {\
            pointer = (void *)elem->u.vPtrElement;\
            changed |= elem->Changed;\
        }\
    }

#define _SET_PROP_ELEM_I(elemn,prop,val,changed)\
    elemn->PropertyID = prop;\
    elemn->u.intElement = val;\
    elemn->Changed = changed;\
    elemn->Type = prtInt;\
    elemn++;


#endif // propertyH
