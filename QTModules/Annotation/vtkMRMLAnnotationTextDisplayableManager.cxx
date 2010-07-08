/*=auto=========================================================================

 Portions (c) Copyright 2005 Brigham and Women's Hospital (BWH) All Rights Reserved.

 See Doc/copyright/copyright.txt
 or http://www.slicer.org/copyright/copyright.txt for details.

 Program:   3D Slicer
 Module:    $RCSfile: vtkMRMLAnnotationTextDisplayableManager.cxx,v $
 Date:      $Date: 2010/10/06 11:42:53 $
 Version:   $Revision: 1.1 $

 =========================================================================auto=*/

#include "vtkObject.h"
#include "vtkObjectFactory.h"
#include "vtkProperty.h"

#include "vtkMath.h"

#include "vtkMRMLAnnotationTextDisplayableManager.h"

#include "vtkLineWidget2.h"
#include "vtkPointHandleRepresentation3D.h"
#include "vtkLineRepresentation.h"
#include "vtkPolygonalSurfacePointPlacer.h"

//#include "vtkSlicerViewerWidget.h"
//#include "vtkSlicerApplication.h"
//#include "vtkSlicerApplicationGUI.h"

#include "vtkMRMLTransformNode.h"
#include "vtkMRMLLinearTransformNode.h"

#include "vtkCamera.h"
#include "vtkRenderer.h"

#include "vtkTextRepresentation.h"
#include "vtkTextWidget.h"
#include "vtkMRMLAnnotationTextNode.h"

//---------------------------------------------------------------------------
vtkStandardNewMacro (vtkMRMLAnnotationTextDisplayableManager );
vtkCxxRevisionMacro ( vtkMRMLAnnotationTextDisplayableManager, "$Revision: 1.1 $");

// Constructors
vtkMRMLAnnotationTextDisplayableManager::vtkMRMLAnnotationTextDisplayableManager()
{

  this->NodeID = NULL;
  this->ViewerWidget = NULL;
  this->Updating3DWidget = 0;

}

// Destructor
vtkMRMLAnnotationTextDisplayableManager::~vtkMRMLAnnotationTextDisplayableManager()
{

  this->NodeID=NULL;
}

//---------------------------------------------------------------------------
/*vtkMRMLAnnotationTextDisplayableManager::vtkMRMLAnnotationTextDisplayableManager ( )
 {
 this->NodeID = NULL;
 this->ViewerWidget = NULL;
 this->Updating3DWidget = 0;
 }*/

//---------------------------------------------------------------------------
/*vtkMRMLAnnotationTextDisplayableManager::~vtkMRMLAnnotationTextDisplayableManager ( )
 {
 this->RemoveMRMLObservers();

 // 3d widgets
 //this->SetMRMLScene ( NULL );
 
 }*/

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::PrintSelf ( ostream& os, vtkIndent indent )
 {
 this->Superclass::PrintSelf ( os, indent );
 }

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::ProcessMRMLEvents(vtkObject *caller,
    unsigned long event, void *callData)
{/*
 this->ProcessingMRMLEvent = event;

 vtkMRMLAnnotationTextNode *callerNode = vtkMRMLAnnotationTextNode::SafeDownCast(caller);
 vtkMRMLAnnotationTextNode *callDataNode =  NULL;
 callDataNode = reinterpret_cast<vtkMRMLAnnotationTextNode *>(callData);
 vtkMRMLScene *callScene = vtkMRMLScene::SafeDownCast(caller);

 // the scene was closed, don't get node removed events so clear up here
 if (callScene != NULL && event == vtkMRMLScene::SceneCloseEvent)
 {
 //vtkDebugMacro("ProcessMRMLEvents: got a scene close event");
 // the lists are already gone from the scene, so need to clear out all the
 // widget properties, can't call remove with a node
 this->RemoveTextWidgets();
 return;
 }
 // first check to see if there was a ruler list node deleted
 else if (callScene != NULL && callScene == this->MRMLScene && event == vtkMRMLScene::NodeRemovedEvent)
 {
 //vtkDebugMacro("ProcessMRMLEvents: got a node deleted event on scene");
 // check to see if it was the current node that was deleted
 if (callData != NULL)
 {
 vtkMRMLAnnotationTextNode *delNode = reinterpret_cast<vtkMRMLAnnotationTextNode*>(callData);
 if (delNode != NULL && delNode->IsA("vtkMRMLAnnotationTextNode"))
 {
 //vtkDebugMacro("A ruler node got deleted " << (delNode->GetID() == NULL ? "null" : delNode->GetID()));
 this->RemoveTextWidget(delNode);
 }
 }
 }
 else if (callScene != NULL && event == vtkMRMLScene::NodeAddedEvent && callDataNode != NULL && vtkMRMLAnnotationTextNode::SafeDownCast ( (vtkObjectBase *)callData ))
 {
 this->Update3DWidget(callDataNode);
 this->RequestRender();
 }
 else if (callerNode != NULL && event == vtkMRMLAnnotationNode::LockModifiedEvent)
 {
 this->UpdateLockUnlock(callerNode);
 }
 else if (callerNode != NULL && event == vtkCommand::ModifiedEvent)
 {
 this->UpdateWidget(callerNode);
 this->RequestRender();
 }
 */
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::Update3DWidget(vtkMRMLAnnotationTextNode *activeNode)
{/*
 if (activeNode == NULL)
 {
 //vtkDebugMacro("Update3DWidget: passed in node is null, returning");
 return;
 }

 if (this->Updating3DWidget)
 {
 //vtkDebugMacro("Already updating 3d widget");
 return;
 }

 vtkTextWidget *widget = this->GetTextWidget(activeNode->GetID());
 if (!widget)
 {
 cout << "No widget found, adding a widget for this one" << endl;
 this->AddTextWidget(activeNode);
 widget = this->GetTextWidget(activeNode->GetID());
 if (!widget)
 {
 //vtkErrorMacro("Error adding a new widget for node " << activeNode->GetID());
 this->Updating3DWidget = 0;
 return;
 }
 }

 this->Updating3DWidget = 1;

 vtkTextRepresentation* textRep = vtkTextRepresentation::New();
 textRep->SetMoving(1);
 textRep->SetText(activeNode->GetText(0));
 widget->SetRepresentation(textRep);
 widget->SetInteractor(this->GetViewerWidget()->GetMainViewer()->GetRenderWindowInteractor());
 widget->On();

 if (activeNode->HasObserver ( vtkCommand::ModifiedEvent, this->MRMLCallbackCommand ) == 0)
 {
 activeNode->AddObserver ( vtkCommand::ModifiedEvent, this->MRMLCallbackCommand );
 }
 if (activeNode->HasObserver ( vtkMRMLTransformableNode::TransformModifiedEvent, this->MRMLCallbackCommand ) == 0)
 {
 activeNode->AddObserver ( vtkMRMLTransformableNode::TransformModifiedEvent, this->MRMLCallbackCommand );
 }
 if (activeNode->HasObserver(vtkMRMLAnnotationNode::LockModifiedEvent, (vtkCommand *)this->MRMLCallbackCommand) != 1)
 {
 activeNode->AddObserver(vtkMRMLAnnotationNode::LockModifiedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }

 this->Updating3DWidget = 0;
 */
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::AddMRMLObservers()
{/*
 // the widget as a whole needs to keep track of ruler nodes in the scene
 if (this->MRMLScene)
 {
 //vtkDebugMacro("AddMRMLObservers: watching for node removed, added, scene close events on the scene");
 if (this->MRMLScene->HasObserver(vtkMRMLScene::NodeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand) != 1)
 {
 this->MRMLScene->AddObserver(vtkMRMLScene::NodeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }
 if (this->MRMLScene->HasObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand) != 1)
 {
 this->MRMLScene->AddObserver(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }
 if (this->MRMLScene->HasObserver(vtkMRMLScene::SceneCloseEvent, (vtkCommand *)this->MRMLCallbackCommand) != 1)
 {
 this->MRMLScene->AddObserver(vtkMRMLScene::SceneCloseEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }
 }*/
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::RemoveMRMLObservers()
{/*
 // remove observers on the ruler nodes
 int nnodes = this->MRMLScene->GetNumberOfNodesByClass("vtkMRMLAnnotationTextNode");
 for (int n=0; n<nnodes; n++)
 {
 vtkMRMLAnnotationTextNode *node = vtkMRMLAnnotationTextNode::SafeDownCast(this->MRMLScene->GetNthNodeByClass(n, "vtkMRMLAnnotationTextNode"));
 if (node->HasObserver(vtkMRMLTransformableNode::TransformModifiedEvent, (vtkCommand *)this->MRMLCallbackCommand))
 {
 node->RemoveObservers(vtkMRMLTransformableNode::TransformModifiedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }
 if (node->HasObserver(vtkCommand::ModifiedEvent, (vtkCommand *)this->MRMLCallbackCommand))
 {
 node->RemoveObservers(vtkCommand::ModifiedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }
 node = NULL;
 }
 //events->Delete();

 if (this->MRMLScene)
 {
 //vtkDebugMacro("RemoveMRMLObservers: stopping watching for node removed, added, scene close events on the scene");
 this->MRMLScene->RemoveObservers(vtkMRMLScene::NodeRemovedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 this->MRMLScene->RemoveObservers(vtkMRMLScene::NodeAddedEvent, (vtkCommand *)this->MRMLCallbackCommand);
 this->MRMLScene->RemoveObservers(vtkMRMLScene::SceneCloseEvent, (vtkCommand *)this->MRMLCallbackCommand);
 }*/
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::AddTextWidget(vtkMRMLAnnotationTextNode *node)
{
  if (!node)
    {
    return;
    }
  if (this->GetTextWidget(node->GetID()) != NULL)
    {
    //vtkDebugMacro("Already have widgets for node " << node->GetID());
    return;
    }

  //vtkTextWidget* c = vtkTextWidget::New();

  //this->TextWidgets[node->GetID()] = c;
}

//---------------------------------------------------------------------------
vtkTextWidget*
vtkMRMLAnnotationTextDisplayableManager::GetTextWidget(const char *nodeID)
{/*
 std::map<std::string, vtkTextWidget *>::iterator iter;
 for (iter = this->TextWidgets.begin(); iter != this->TextWidgets.end(); iter++)
 {
 if (iter->first.c_str() && !strcmp(iter->first.c_str(), nodeID))
 {
 return iter->second;
 }
 }*/
  return NULL;
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::RemoveTextWidget(vtkMRMLAnnotationTextNode *node)
{/*
 if (!node)
 {
 return;
 }
 if (this->GetTextWidget(node->GetID()) != NULL)
 {
 this->TextWidgets[node->GetID()]->Delete();
 this->TextWidgets.erase(node->GetID());
 }*/
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::RemoveTextWidgets()
{/*
 int nnodes = this->MRMLScene->GetNumberOfNodesByClass("vtkMRMLAnnotationTextNode");
 //vtkDebugMacro("RemoveTextWidgets: have " << nnodes << " ruler  nodes in the scene, " << this->TextWidgets.size() << " widgets defined already");

 if (nnodes == 0)
 {
 // the scene was closed, all the nodes are gone, so do this w/o reference
 // to the nodes
 //vtkDebugMacro("RemoveDistanceWidgets: no ruler nodes in scene, removing ruler widgets w/o reference to nodes");
 std::map<std::string, vtkTextWidget *>::iterator iter;
 for (iter = this->TextWidgets.end(); iter != this->TextWidgets.end(); iter++)
 {
 //vtkDebugMacro("RemoveTextWidgets: deleting and erasing " << iter->first);
 iter->second->Delete();
 this->TextWidgets.erase(iter);
 }
 this->TextWidgets.clear();
 }
 else
 {
 for (int n=0; n<nnodes; n++)
 {
 vtkMRMLAnnotationTextNode *node = vtkMRMLAnnotationTextNode::SafeDownCast(this->MRMLScene->GetNthNodeByClass(n, "vtkMRMLAnnotationTextNode"));
 if (node)
 {
 this->RemoveTextWidget(node);
 }
 }
 }*/
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::RequestRender()
{
  //this->GetViewerWidget()->RequestRender();
}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::Render()
{
  //this->GetViewerWidget()->Render();
}

//---------------------------------------------------------------------------
/*void vtkMRMLAnnotationTextDisplayableManager::SetViewerWidget ( vtkSlicerViewerWidget *viewerWidget )
 {
 if (this->ViewerWidget != NULL)
 {
 // TODO: figure out if this is necessary
 this->RemoveTextWidgets();
 if (this->ViewerWidget->HasObserver(
 vtkSlicerViewerWidget::ActiveCameraChangedEvent,
 this->GUICallbackCommand) == 1)
 {
 this->ViewerWidget->RemoveObservers(
 vtkSlicerViewerWidget::ActiveCameraChangedEvent,
 (vtkCommand *)this->GUICallbackCommand);
 }
 }

 this->ViewerWidget = viewerWidget;

 if (this->ViewerWidget)
 {
 this->ViewerWidget->AddObserver(
 vtkSlicerViewerWidget::ActiveCameraChangedEvent,
 (vtkCommand *)this->GUICallbackCommand);
 }

 //vtkDebugMacro("SetViewerWidget: Updating any widget interactors");
 this->UpdateWidgetInteractors();
 }*/

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::UpdateWidgetInteractors()
{
  /*bool isNull = false;
   if (this->GetViewerWidget() == NULL ||
   this->GetViewerWidget()->GetMainViewer() == NULL ||
   this->GetViewerWidget()->GetMainViewer()->GetRenderWindowInteractor() == NULL)
   {
   //vtkDebugMacro("UpdateSplineWidgetInteractors: viewer widget or main viewer are null, resetting to null");
   isNull = true;
   }

   std::map<std::string, vtkTextWidget *>::iterator iter;
   for (iter = this->TextWidgets.begin(); iter !=  this->TextWidgets.end(); iter++)
   {
   if (iter->second)
   {
   if (isNull)
   {
   iter->second->SetInteractor(NULL);
   }
   else
   {
   iter->second->SetInteractor(this->GetViewerWidget()->GetMainViewer()->GetRenderWindowInteractor());
   }
   // now update the visibility for the ruler
   vtkMRMLAnnotationTextNode *splineNode = NULL;
   if (this->GetMRMLScene())
   {
   vtkMRMLNode *node = this->GetMRMLScene()->GetNodeByID(iter->first.c_str());
   if (node)
   {
   splineNode = vtkMRMLAnnotationTextNode::SafeDownCast(node);
   }
   }
   if (splineNode != NULL)
   {
   //this->Update3DWidgetVisibility(roiNode);
   }
   }
   }*/
}

void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneClosingEvent(vtkMRMLScene* /*scene*/)
{
}
void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneCloseEvent(vtkMRMLScene* /*scene*/)
{
}
void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneLoadStartEvent(vtkMRMLScene* /*scene*/)
{
}
void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneLoadEndEvent(vtkMRMLScene* /*scene*/)
{
}
void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneRestoredEvent(vtkMRMLScene* /*scene*/)
{
}
void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneNodeAddedEvent(vtkMRMLScene* /*scene*/,
    vtkMRMLNode* /*node*/)
{
}
void vtkMRMLAnnotationTextDisplayableManager::OnMRMLSceneNodeRemovedEvent(vtkMRMLScene* /*scene*/,
    vtkMRMLNode* /*node*/)
{
}

void vtkMRMLAnnotationTextDisplayableManager::UpdateFromMRML()
{

}

//---------------------------------------------------------------------------
void vtkMRMLAnnotationTextDisplayableManager::UpdateLockUnlock(vtkMRMLAnnotationTextNode* textNode)
{
  if (textNode == NULL)
    {
    return;
    }

  vtkTextWidget *widget = this->GetTextWidget(textNode->GetID());
  if (!widget)
    {
    cout << "No distance widget found, adding a distance widget for this one" << endl;
    return;
    }

  if (textNode->GetLocked())
    {
    widget->ProcessEventsOff();
    }
  else
    {
    widget->ProcessEventsOn();
    }
}

void vtkMRMLAnnotationTextDisplayableManager::UpdateWidget(vtkMRMLAnnotationTextNode *activeNode)
{
  vtkTextWidget *widget = this->GetTextWidget(activeNode->GetID());
  if (widget == NULL)
    {
    return;
    }

  vtkTextRepresentation* rep = vtkTextRepresentation::SafeDownCast(widget->GetRepresentation());
  rep->SetText(activeNode->GetText(0));

  if (activeNode->GetVisible())
    {
    widget->EnabledOn();
    }
  else
    {
    widget->EnabledOff();
    }
}
