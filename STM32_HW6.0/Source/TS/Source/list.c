/* list.c -- functions supporting list operations */
#include <stdio.h>
#include <stdlib.h>
#include "list.h"

/* local function prototype */
static void CopyToNode(Item item, Node * pnode);
static Item CopyToItem(List * plist);


/* interface functions   */
/* set the list to empty */
void InitializeList(List * plist)
{
    * plist = NULL;
}

/* returns true if list is empty */
bool ListIsEmpty(const List * plist)
{
    if (*plist == NULL)
        return true;
    else
        return false;
}

/* returns true if list is full */
bool ListIsFull(const List * plist)
{
    Node * pt;
    bool full;

    pt = (Node *) malloc(sizeof(Node));
    if (pt == NULL)
        full = true;
    else
        full = false;
    free(pt);
 
    return full;
}

/* returns number of nodes */
unsigned int ListItemCount(const List * plist)
{
    unsigned int count = 0;
    Node * pnode = *plist;    /* set to start of list */

    while (pnode != NULL)
    {
        ++count;
        pnode = pnode->next;  /* set to next node     */
    }
    
    return count;
} 

/* creates node to hold item and adds it to the end of */
/* the list pointed to by plist (slow implementation)  */
bool AddItem(Item item, List * plist)
{
    Node * pnew = NULL;
    Node * scan = *plist;

    pnew = (Node *) malloc(sizeof(Node));
    if (pnew == NULL)
	{
        return false;     /* quit function on failure  */
	}
//	memset(pnew, 0, sizeof(Node));
    CopyToNode(item, pnew);
    pnew->next = NULL;
    if (scan == NULL)          /* empty list, so place */
        *plist = pnew;         /* pnew at head of list */
    else
    {
        while (scan->next != NULL)
            scan = scan->next;  /* find end of list    */
        scan->next = pnew;      /* add pnew to end     */
    }
    return true;
}

Item  GetItem(List  * plist)
{
    Node * psave;
	Item item;
	
	item.events = 0;
    if (*plist != NULL)
    {
		item = CopyToItem(plist);
        psave = (*plist)->next; /* save address of next node */
        free(*plist);           /* free current node         */
        *plist = psave;         /* advance to next node      */
		return  item;
    }	
	else
	{
		return  item;
	}
}

/* visit each node and execute function pointed to by pfun */
void Traverse  (const List * plist, void (* pfun)(Item item) )
{
    Node * pnode = *plist;    /* set to start of list   */

    while (pnode != NULL)
    {
        (*pfun)(pnode->item); /* apply function to item */
        pnode = pnode->next;  /* advance to next item   */
    }
}

/* free memory allocated by malloc() */
/* set list pointer to NULL          */
void EmptyTheList(List * plist)
{
    Node * psave;

    while (*plist != NULL)
    {
        psave = (*plist)->next; /* save address of next node */
        free(*plist);           /* free current node         */
        *plist = psave;         /* advance to next node      */
    }
}
/* local function definition  */
/* copies an item into a node */
static void CopyToNode(Item item, Node * pnode)
{
	//printf("bef evt= %x, %x\r\n", item.events,pnode);
    pnode->item.events = item.events;  /* structure copy */
	//printf("cop evt= %x\r\n", pnode->item.events);
}
/* local function definition  */
/* copies a node into an item */
static Item CopyToItem(List * plist)
{
	 Item item;
	 Node * pnode = *plist;
     item = pnode->item;  /* structure copy */
	 return  item;
}
