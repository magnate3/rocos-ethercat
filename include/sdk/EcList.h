/*-----------------------------------------------------------------------------
 * CList.h                  header file
 * Copyright                acontis technologies GmbH, Weingarten, Germany
 * Response                 Stefan Zintgraf
 * Description              
 *---------------------------------------------------------------------------*/

#ifndef INC_LIST
#define INC_LIST

#ifdef _MSC_VER
#pragma warning (disable: 4710)
#endif

/*-INCLUDES------------------------------------------------------------------*/

/*-EXTERNALS-----------------------------------------------------------------*/

/*-TYPEDEFS/ENUMS------------------------------------------------------------*/

template<class CLIST_T_DATA>
    class CNode
    {
    public:
        CNode(){};
        ~CNode(){};
        CNode* pNext;
        CNode* pPrev;
        CLIST_T_DATA   data;
    };

template<class CLIST_T_DATA, class CLIST_T_ARG> 
class CList
{
    
/*-EMBEDDED CLASSES----------------------------------------------------------*/
public:
    struct CNode
    {
        CNode* pNext;
        CNode* pPrev;
        CLIST_T_DATA data;
    };

protected:
private:
    
    
/*-CONSTRUCTORS/DESTRUCTORS--------------------------------------------------*/
public:
    /* -constructors/destructors/initialization */
    CList( ) 
    {   
        m_pFirstNode = EC_NULL;
        m_pLastNode = EC_NULL;
        m_nNumNodes = 0;
    };
    virtual ~CList();
protected:
private:
    
    
/*-ATTRIBUTES----------------------------------------------------------------*/
public:

protected:
    CNode*  m_pFirstNode;
    CNode*  m_pLastNode;
    EC_T_INT   m_nNumNodes;
private:

/*-METHODS-------------------------------------------------------------------*/
public:
    /* count of elements */
    EC_T_INT GetCount( void ) const    { return m_nNumNodes; }
    EC_T_BOOL IsEmpty( void ) const    { return m_nNumNodes == 0; }

    /* get head or tail (and remove it) - don't call on empty list ! */
    CLIST_T_DATA RemoveTail();

    /* add before head or after tail */
    EC_T_VOID AddHead( CLIST_T_ARG newElement );
    EC_T_VOID AddTail( CLIST_T_ARG newElement );

    /* iteration */
    CNode* GetFirstNode() const    { return m_pFirstNode; }

    CLIST_T_DATA GetNext(CNode*& pNode) const; /* return *Position++ */

    /* getting/modifying an element at a given position */
    CLIST_T_DATA& GetAt(CNode* position);

    void RemoveAt(CNode* pOldNode);
    EC_T_BOOL FindAndDelete(CLIST_T_ARG ElementToFind );
    EC_T_VOID Find( CNode*& pNode, CLIST_T_ARG ElementToFind );

protected:
private:
};

/*-INLINE METHODS------------------------------------------------------------*/

/********************************************************************************/
/** \brief AddHead
*
* \return
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
CList<CLIST_T_DATA, CLIST_T_ARG>::~CList()
{
	CNode* cur = EC_NULL;
	
	while(m_pFirstNode)
	{
		cur = m_pFirstNode;
		m_pFirstNode = m_pFirstNode->pNext;

		SafeDelete(cur);
	};
	
}

/********************************************************************************/
/** \brief AddHead
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
EC_T_VOID CList<CLIST_T_DATA, CLIST_T_ARG>::AddHead(CLIST_T_ARG newElement)
{
    CNode* pNewNode = EC_NEW(CNode);
    if (EC_NULL != pNewNode)
    {
        pNewNode->data = newElement;
        if( m_pFirstNode == EC_NULL )
        {
            m_pFirstNode = m_pLastNode = pNewNode;
            pNewNode->pNext = EC_NULL;
            pNewNode->pPrev = EC_NULL;
        }
        else
        {
            pNewNode->pNext = m_pFirstNode;
            m_pFirstNode->pPrev = pNewNode;
            m_pFirstNode = pNewNode;
            pNewNode->pPrev = EC_NULL;
        }
        m_nNumNodes++;
    }
    return;
}

/********************************************************************************/
/** \brief AddTail
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
EC_T_VOID CList<CLIST_T_DATA, CLIST_T_ARG>::AddTail(CLIST_T_ARG newElement)
{
    CNode* pNewNode = EC_NEW(CNode);
    if (EC_NULL != pNewNode)
    {
        pNewNode->data = newElement;
        if( m_pLastNode == EC_NULL )
        {
            m_pFirstNode = m_pLastNode = pNewNode;
            pNewNode->pNext = EC_NULL;
            pNewNode->pPrev = EC_NULL;
        }
        else
        {
            pNewNode->pPrev = m_pLastNode;
            m_pLastNode->pNext = pNewNode;
            m_pLastNode = pNewNode;
            pNewNode->pNext = EC_NULL;
        }
        m_nNumNodes++;
    }
    return;
}

/********************************************************************************/
/** \brief RemoveTail
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
CLIST_T_DATA CList<CLIST_T_DATA, CLIST_T_ARG>::RemoveTail()
{
    CNode* pOldNode = m_pLastNode;
    if( m_nNumNodes == 1 )
    {
        /* DBG_ASSERT( m_pFirstNode == m_pLastNode ); */
        m_pFirstNode = EC_NULL;
        m_pLastNode = EC_NULL;
    }
    else
    {
        m_pLastNode = pOldNode->pPrev;
        m_pLastNode->pNext = EC_NULL;
    }
    m_nNumNodes--;
    CLIST_T_DATA data = pOldNode->data;
    SafeDelete(pOldNode);
    
    return data; 
}

/********************************************************************************/
/** \brief GetAt
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
CLIST_T_DATA& CList<CLIST_T_DATA, CLIST_T_ARG>::GetAt(CNode* pCurNode)
{
    return pCurNode->data;
}

/********************************************************************************/
/** \brief RemoveAt
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
void CList<CLIST_T_DATA, CLIST_T_ARG>::RemoveAt(CNode* pOldNode)
{
    /* remove pOldNode from list */
    if (pOldNode == m_pFirstNode)
    {
        m_pFirstNode = pOldNode->pNext;
    }
    else
    {
        pOldNode->pPrev->pNext = pOldNode->pNext;
    }
    if (pOldNode == m_pLastNode)
    {
        m_pLastNode = pOldNode->pPrev;
    }
    else
    {
        pOldNode->pNext->pPrev = pOldNode->pPrev;
    }
    SafeDelete(pOldNode);
    m_nNumNodes--;
}

/********************************************************************************/
/** \brief FindAndDelete
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
EC_T_BOOL CList<CLIST_T_DATA, CLIST_T_ARG>::FindAndDelete(CLIST_T_ARG ElementToFind )
{
EC_T_BOOL bFound = EC_FALSE;

    if( m_pFirstNode != EC_NULL )
    {
        CNode* pNode = m_pFirstNode;
        for( EC_T_INT nListIndex = 0; nListIndex < m_nNumNodes; nListIndex++ )
        {
            if( pNode->data == ElementToFind )
            {
                bFound = EC_TRUE;
                break;
            }
            pNode = (CNode*)pNode->pNext;
        }
        if( bFound )
        {
            RemoveAt( pNode );    /*  remove from list */
        }
    }
    return bFound;
}

/********************************************************************************/
/** \brief Find
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
EC_T_VOID CList<CLIST_T_DATA, CLIST_T_ARG>::Find(CNode*& pNode, CLIST_T_ARG ElementToFind )
{
    EC_T_BOOL bFound = EC_FALSE;
    
    pNode = EC_NULL;
    
    if( m_pFirstNode != EC_NULL )
    {
        pNode = m_pFirstNode;
        for( EC_T_INT nListIndex = 0; nListIndex < m_nNumNodes; nListIndex++ )
        {
            if( pNode->data == ElementToFind )
            {
                bFound = EC_TRUE;
                break;
            }
            pNode = (CNode*)pNode->pNext;
        }
        if( !bFound )
        {
            pNode = EC_NULL;
        }
    }

    return;
}

/********************************************************************************/
/** \brief GetNext
*
* \return 
*/
template<class CLIST_T_DATA, class CLIST_T_ARG>
CLIST_T_DATA CList<CLIST_T_DATA, CLIST_T_ARG>::GetNext(CNode*& pCurNode) const /* return *Position++ */
{ 
    CLIST_T_DATA* pElement;

    CNode* pNode = (CNode*) pCurNode;
    pCurNode = (CNode*) pNode->pNext;
    pElement = &(pNode->data); 
    return *pElement; 
}

#endif /* INC_LIST */


/*-END OF SOURCE FILE--------------------------------------------------------*/
