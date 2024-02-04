/*==============================================================================

  Program: 3D Slicer

  Portions (c) Copyright Brigham and Women's Hospital (BWH) All Rights Reserved.

  See COPYRIGHT.txt
  or http://www.slicer.org/copyright/copyright.txt for details.

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  This file was originally developed by Andras Lasso and Franklin King at
  PerkLab, Queen's University and was supported through the Applied Cancer
  Research Unit program of Cancer Care Ontario with funds provided by the
  Ontario Ministry of Health and Long-Term Care.

==============================================================================*/


// MRMLDisplayableManager includes
#include "vtkMRMLLinearTransformsDisplayableManager3D.h"

#include "vtkSlicerTransformLogic.h"

// MRML includes
#include <vtkEventBroker.h>
#include <vtkMRMLScene.h>
#include <vtkMRMLTransformDisplayNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkMRMLViewNode.h>

// VTK includes
#include <vtkCallbackCommand.h>
#include <vtkObjectFactory.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include "vtkMRMLInteractionNode.h"
#include "vtkSlicerLinearTransformWidget.h"
#include "vtkSlicerLinearTransformWidgetRepresentation.h"

const double DEFAULT_SCALE[3] = {100.0, 100.0, 100.0};

//---------------------------------------------------------------------------
vtkStandardNewMacro ( vtkMRMLLinearTransformsDisplayableManager3D );

//---------------------------------------------------------------------------
// vtkMRMLLinearTransformsDisplayableManager3D Callback
class vtkLinearTransformWidgetCallback : public vtkCommand
{
public:
  static vtkLinearTransformWidgetCallback *New()
  { return new vtkLinearTransformWidgetCallback; }

  vtkLinearTransformWidgetCallback() = default;

  void Execute (vtkObject *vtkNotUsed(caller), unsigned long event, void *vtkNotUsed(callData)) override
  {
    if ((event == vtkCommand::StartInteractionEvent) || (event == vtkCommand::EndInteractionEvent) || (event == vtkCommand::InteractionEvent))
      {
      // sanity checks
      if (!this->DisplayableManager)
        {
        return;
        }
      if (!this->Node)
        {
        return;
        }
      if (!this->Widget)
        {
        return;
        }
      // sanity checks end
      }

    if (event == vtkCommand::StartInteractionEvent)
      {
      // save the state of the node when starting interaction
      if (this->Node->GetScene())
        {
        this->Node->GetScene()->SaveStateForUndo();
        }
      }
  }

  void SetWidget(vtkAbstractWidget *w)
    {
    this->Widget = w;
    }
  void SetNode(vtkMRMLTransformNode *n)
    {
    this->Node = n;
    }
  void SetDisplayableManager(vtkMRMLLinearTransformsDisplayableManager3D* dm)
    {
    this->DisplayableManager = dm;
    }

  vtkAbstractWidget * Widget{nullptr};
  vtkMRMLTransformNode* Node{nullptr};
  vtkMRMLLinearTransformsDisplayableManager3D* DisplayableManager{nullptr};
};

//---------------------------------------------------------------------------
class vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
{
public:

  vtkInternal(vtkMRMLLinearTransformsDisplayableManager3D* external);
  ~vtkInternal();

  /*struct Pipeline
    {
    vtkSmartPointer<vtkBoxWidget2> Widget;
    vtkSmartPointer<vtkTransform> WidgetDisplayTransform;
    bool UpdateWidgetBounds;
    };*/

  //typedef std::map < vtkMRMLTransformDisplayNode*, Pipeline* > PipelinesCacheType;
  //PipelinesCacheType DisplayPipelines;

  typedef std::map < vtkMRMLTransformNode*, std::set< vtkMRMLTransformDisplayNode* > > TransformToDisplayCacheType;
  TransformToDisplayCacheType TransformToDisplayNodes;

  //typedef std::map < vtkBoxWidget2*, vtkMRMLTransformDisplayNode* > WidgetToNodeMapType;
  //WidgetToNodeMapType WidgetMap;

  /// Map of vtkWidget indexed using associated node ID
  typedef std::map < vtkSmartPointer<vtkMRMLTransformDisplayNode>, vtkSlicerLinearTransformWidget* > DisplayNodeToWidgetType;
  typedef std::map < vtkSmartPointer<vtkMRMLTransformDisplayNode>, vtkSlicerLinearTransformWidget* >::iterator DisplayNodeToWidgetIt;
  DisplayNodeToWidgetType TransformDisplayNodesToWidgets;  // display nodes with widgets assigned

  // Transforms
  void AddTransformNode(vtkMRMLTransformNode* displayableNode);
  void RemoveTransformNode(vtkMRMLTransformNode* displayableNode);
  //void UpdateDisplayableTransforms(vtkMRMLTransformNode *node, bool);

  // Display Nodes
  void AddDisplayNode(vtkMRMLTransformNode*, vtkMRMLTransformDisplayNode*);
  //void UpdateDisplayNode(vtkMRMLTransformDisplayNode* displayNode);
  //void UpdateDisplayNodePipeline(vtkMRMLTransformDisplayNode*, Pipeline*);
  void RemoveDisplayNode(vtkMRMLTransformDisplayNode* displayNode);
  //void SetTransformDisplayProperty(vtkMRMLTransformDisplayNode *displayNode, vtkActor* actor);

  // Widget
  //void UpdateWidgetDisplayTransform(Pipeline*, vtkMRMLTransformNode*);
  //void UpdateWidgetFromNode(vtkMRMLTransformDisplayNode*, vtkMRMLTransformNode*, Pipeline*);
  //void UpdateNodeFromWidget(vtkBoxWidget2*);

  // New Widget
  vtkSlicerLinearTransformWidget* GetWidget(vtkMRMLTransformDisplayNode* node);
  vtkSlicerLinearTransformWidget* CreateWidget(vtkMRMLTransformDisplayNode* displayNode);

  // Observations
  void AddObservations(vtkMRMLTransformNode* node);
  void RemoveObservations(vtkMRMLTransformNode* node);
  void AddDisplayObservations(vtkMRMLTransformDisplayNode* node);
  void RemoveDisplayObservations(vtkMRMLTransformDisplayNode* node);

  // Helper functions
  bool UseDisplayNode(vtkMRMLTransformDisplayNode* displayNode);
  bool UseDisplayableNode(vtkMRMLTransformNode* node);
  void ClearDisplayableNodes();

private:
  vtkMRMLLinearTransformsDisplayableManager3D* External;
  bool AddingTransformNode;
};

//---------------------------------------------------------------------------
// vtkInternal methods

//---------------------------------------------------------------------------
vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::vtkInternal(vtkMRMLLinearTransformsDisplayableManager3D * external)
: External(external)
, AddingTransformNode(false)
{
}

//---------------------------------------------------------------------------
vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::~vtkInternal()
{
  this->ClearDisplayableNodes();
}

//---------------------------------------------------------------------------
bool vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::UseDisplayNode(vtkMRMLTransformDisplayNode* displayNode)
{
   // allow nodes to appear only in designated viewers
  if (displayNode && !displayNode->IsDisplayableInView(this->External->GetMRMLDisplayableNode()->GetID()))
    {
    return false;
    }

  // Check whether DisplayNode should be shown in this view
  bool use = displayNode && displayNode->IsA("vtkMRMLTransformDisplayNode");

  return use;
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::AddTransformNode(vtkMRMLTransformNode* node)
{
  if (this->AddingTransformNode)
    {
    return;
    }
  // Check if node should be used
  if (!this->UseDisplayableNode(node))
    {
    return;
    }

  this->AddingTransformNode = true;
  // Add Display Nodes
  int nnodes = node->GetNumberOfDisplayNodes();

  this->AddObservations(node);

  for (int i=0; i<nnodes; i++)
    {
    vtkMRMLTransformDisplayNode *dnode = vtkMRMLTransformDisplayNode::SafeDownCast(node->GetNthDisplayNode(i));
    if ( this->UseDisplayNode(dnode) )
      {
      this->TransformToDisplayNodes[node].insert(dnode);
      this->AddDisplayNode( node, dnode );
      }
    }
  this->AddingTransformNode = false;
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::RemoveTransformNode(vtkMRMLTransformNode* node)
{
  if (!node)
    {
    return;
    }
  vtkInternal::TransformToDisplayCacheType::iterator displayableIt =
    this->TransformToDisplayNodes.find(node);
  if(displayableIt == this->TransformToDisplayNodes.end())
    {
    return;
    }

  std::set<vtkMRMLTransformDisplayNode *> dnodes = displayableIt->second;
  std::set<vtkMRMLTransformDisplayNode *>::iterator diter;
  for ( diter = dnodes.begin(); diter != dnodes.end(); ++diter)
    {
    this->RemoveDisplayNode(*diter);
    }
  this->RemoveObservations(node);
  this->TransformToDisplayNodes.erase(displayableIt);
}

//---------------------------------------------------------------------------
/*void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::UpdateDisplayableTransforms(vtkMRMLTransformNode* mNode, bool updateBounds)
{
  // Update the pipeline for all tracked DisplayableNode
  PipelinesCacheType::iterator pipelinesIter;
  std::set< vtkMRMLTransformDisplayNode* > displayNodes = this->TransformToDisplayNodes[mNode];
  std::set< vtkMRMLTransformDisplayNode* >::iterator dnodesIter;
  for ( dnodesIter = displayNodes.begin(); dnodesIter != displayNodes.end(); dnodesIter++ )
    {
    if ( ((pipelinesIter = this->DisplayPipelines.find(*dnodesIter)) != this->DisplayPipelines.end()) )
      {
      Pipeline* pipeline = pipelinesIter->second;
      pipeline->UpdateWidgetBounds |= updateBounds;
      this->UpdateDisplayNodePipeline(pipelinesIter->first, pipeline);
      }
    }
}*/

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::RemoveDisplayNode(vtkMRMLTransformDisplayNode* displayNode)
{
  if (!displayNode)
  {
    return;
  }
  this->RemoveDisplayObservations(displayNode);

  DisplayNodeToWidgetIt displayNodeIt = this->TransformDisplayNodesToWidgets.find(displayNode);
  if (displayNodeIt == this->TransformDisplayNodesToWidgets.end())
  {
    return;
  }
  vtkSlicerLinearTransformWidget* widget = (displayNodeIt->second);
  if (!widget)
  {
    return;
  }
  widget->SetRenderer(nullptr);
  widget->SetRepresentation(nullptr);
  widget->Delete();
  this->TransformDisplayNodesToWidgets.erase(displayNode);

}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::AddDisplayNode(vtkMRMLTransformNode* mNode, vtkMRMLTransformDisplayNode* displayNode)
{
  if (!mNode || !displayNode)
    {
    return;
    }

  // Do not add the display node if displayNodeIt is already associated with a widget object.
  // This happens when a segmentation node already associated with a display node
  // is copied into an other (using vtkMRMLNode::Copy()) and is added to the scene afterward.
  // Related issue are #3428 and #2608
  DisplayNodeToWidgetIt displayNodeIt
    = this->TransformDisplayNodesToWidgets.find(displayNode);
  if (displayNodeIt != this->TransformDisplayNodesToWidgets.end())
  {
    return;
  }

  //todo need this?
  this->AddDisplayObservations(displayNode);

  // There should not be a widget for the new node
  if (this->GetWidget(displayNode) != nullptr)
  {
    vtkDebugWithObjectMacro(displayNodeIt->first,"vtkMRMLTransformDisplayableManager3D: A widget is already associated to this node");
    return;
  }

  int wasModified = 0;
  if (displayNode->GetDisplayableNode())
  {
    // Prevent potential recursive calls during UpdateFromMRML call before the new widget is stored
    // in TransformDisplayNodesToWidgets.
    wasModified = displayNode->GetDisplayableNode()->StartModify();
  }

  vtkSlicerLinearTransformWidget* newWidget = CreateWidget(displayNode);
  if (!newWidget)
  {
    vtkWarningWithObjectMacro(newWidget,"vtkMRMLLinearDisplayableManager3D: Failed to create widget");
    return;
  }

  // record the mapping between node and widget in the helper
  this->TransformDisplayNodesToWidgets[displayNode] = newWidget;

  // Build representation
  newWidget->UpdateFromMRML(displayNode, 0); // no specific event triggers full rebuild

  if (displayNode->GetDisplayableNode())
  {
    displayNode->GetDisplayableNode()->EndModify(wasModified);
  }
}

//---------------------------------------------------------------------------
/*void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::UpdateDisplayNode(vtkMRMLTransformDisplayNode* displayNode)
{
  // If the DisplayNode already exists, just update.
  //   otherwise, add as new node

  if (!displayNode)
    {
    return;
    }
  PipelinesCacheType::iterator it;
  it = this->DisplayPipelines.find(displayNode);
  if (it != this->DisplayPipelines.end())
    {
    this->UpdateDisplayNodePipeline(displayNode, it->second);
    }
  else
    {
    this->AddTransformNode( vtkMRMLTransformNode::SafeDownCast(displayNode->GetDisplayableNode()) );
    }
}*/

//---------------------------------------------------------------------------
/*void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
::UpdateWidgetDisplayTransform(
  Pipeline* pipeline, vtkMRMLTransformNode* transformNode)
{
  // WidgetDisplayTransform is in charge of moving the widget around
  // the bounding box of the objects under the transform, wherever they
  // are.
  assert(pipeline && transformNode);

  std::vector<vtkMRMLDisplayableNode*> transformedNodes;
  vtkSlicerTransformLogic::GetTransformedNodes(
    this->External->GetMRMLScene(), transformNode, transformedNodes, true);

  bool validBounds = false;
  double bounds[6];
  if (transformedNodes.size() > 0)
    {
    vtkSlicerTransformLogic::GetNodesBounds(transformedNodes, bounds);
    validBounds =
      (bounds[0] <= bounds[1] || bounds[2] <= bounds[3] || bounds[4] <= bounds[5]);
    }

  if (validBounds)
    {
    // Get the bounding box around the UNTRANSFORMED objects so we have
    // the actual box around the object.
    double center[3], scales[3];
    for (int i = 0; i < 3; ++i)
      {
      double scale = 0.5*(bounds[2*i + 1] - bounds[2*i]);
      center[i] = bounds[2*i] + scale;
      scales[i] = 4*scale;
      }

    pipeline->WidgetDisplayTransform->Identity();
    pipeline->WidgetDisplayTransform->Translate(center);
    pipeline->WidgetDisplayTransform->Scale(scales);
    }
  else
    {
    // No objects, just add a default scaling so the widget can be interacted
    // with more easily.
    pipeline->WidgetDisplayTransform->Identity();
    pipeline->WidgetDisplayTransform->Scale(DEFAULT_SCALE);
    }
}*/

//---------------------------------------------------------------------------
/*void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
::UpdateWidgetFromNode(vtkMRMLTransformDisplayNode* displayNode,
                       vtkMRMLTransformNode* transformNode,
                       Pipeline* pipeline)
{
  assert(displayNode && transformNode && pipeline);

  if (pipeline->UpdateWidgetBounds)
    {
    this->UpdateWidgetDisplayTransform(pipeline, transformNode);
    pipeline->UpdateWidgetBounds = false;
    }

  vtkNew<vtkMatrix4x4> toWorldMatrix;
  transformNode->GetMatrixTransformToWorld(toWorldMatrix.GetPointer());
  vtkNew<vtkTransform> widgetTransform;
  widgetTransform->Concatenate(toWorldMatrix.GetPointer());
  widgetTransform->Concatenate(pipeline->WidgetDisplayTransform);

  vtkBoxRepresentation* representation =
    vtkBoxRepresentation::SafeDownCast(pipeline->Widget->GetRepresentation());

  representation->SetTransform(widgetTransform.GetPointer());

  pipeline->Widget->SetTranslationEnabled(
    displayNode->GetEditorTranslationEnabled());
  pipeline->Widget->SetRotationEnabled(
    displayNode->GetEditorRotationEnabled());
  pipeline->Widget->SetScalingEnabled(
    displayNode->GetEditorScalingEnabled());
  pipeline->Widget->SetMoveFacesEnabled(
    displayNode->GetEditorScalingEnabled());
}*/

//---------------------------------------------------------------------------
/*void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
::UpdateNodeFromWidget(vtkBoxWidget2* widget)
{
  assert(widget);
  vtkMRMLTransformDisplayNode* displayNode = this->WidgetMap[widget];
  assert(displayNode);
  Pipeline* pipeline = this->DisplayPipelines[displayNode];
  vtkMRMLTransformNode* node =
    vtkMRMLTransformNode::SafeDownCast(displayNode->GetDisplayableNode());

  vtkBoxRepresentation* representation =
    vtkBoxRepresentation::SafeDownCast(widget->GetRepresentation());

  vtkNew<vtkTransform> widgetTransform;
  representation->GetTransform(widgetTransform.GetPointer());

  vtkNew<vtkTransform> toParent;
  vtkMRMLTransformNode* parentNode = node->GetParentTransformNode();
  if (parentNode)
    {
    vtkNew<vtkMatrix4x4> worldFromParentMatrix;
    parentNode->GetMatrixTransformFromWorld(worldFromParentMatrix.GetPointer());
    toParent->Concatenate(worldFromParentMatrix.GetPointer());
    }
  toParent->Concatenate(widgetTransform.GetPointer());
  toParent->Concatenate(pipeline->WidgetDisplayTransform->GetLinearInverse());

  node->SetMatrixTransformToParent(toParent->GetMatrix());
}*/

vtkSlicerLinearTransformWidget* vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::GetWidget(
  vtkMRMLTransformDisplayNode* node)
{
  if (!node)
  {
    return nullptr;
  }

  // Make sure the map contains a vtkWidget associated with this node
  DisplayNodeToWidgetIt it = this->TransformDisplayNodesToWidgets.find(node);
  if (it == this->TransformDisplayNodesToWidgets.end())
  {
    return nullptr;
  }

  return it->second;
}

vtkSlicerLinearTransformWidget* vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::CreateWidget(
  vtkMRMLTransformDisplayNode* displayNode)
{
  vtkMRMLTransformNode* transformNode = vtkMRMLTransformNode::SafeDownCast(displayNode->GetDisplayableNode());
  if (!transformNode)
  {
    vtkErrorWithObjectMacro(transformNode, "CreateWidget: invalid Transform logic.");
    return nullptr;
  }

  // Create a widget 
  vtkSlicerLinearTransformWidget* widget = vtkSlicerLinearTransformWidget::New();

  if (!widget)
  {
    vtkErrorWithObjectMacro(widget,"vtkMRMLTransformDisplayableManager::CreateWidget failed: cannot instantiate widget for LinearTransform " );
    return nullptr;
  }

  vtkMRMLAbstractViewNode* viewNode = vtkMRMLAbstractViewNode::SafeDownCast(this->External->GetMRMLDisplayableNode());
  vtkRenderer* renderer = this->External->GetRenderer();
  widget->SetMRMLApplicationLogic(this->External->GetMRMLApplicationLogic());
  widget->CreateDefaultRepresentation(displayNode, viewNode, renderer);
  return widget;
}

//---------------------------------------------------------------------------
/*
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
::UpdateDisplayNodePipeline(
  vtkMRMLTransformDisplayNode* displayNode,
  Pipeline* pipeline)
{
  if (!displayNode || !pipeline)
    {
    return;
    }

  vtkMRMLTransformNode* transformNode =
    vtkMRMLTransformNode::SafeDownCast(displayNode->GetDisplayableNode());
  if (transformNode==nullptr)
    {
    pipeline->Widget->SetEnabled(false);
    return;
    }

  if (!transformNode->IsLinear())
    {
    vtkDebugWithObjectMacro(transformNode, "Cannot show interactive widget: Transform is not linear");
    pipeline->Widget->SetEnabled(false);
    return;
    }

  bool visible = displayNode->GetEditorVisibility();
  pipeline->Widget->SetEnabled(visible);
  if (visible)
    {
    this->UpdateWidgetFromNode(displayNode, transformNode, pipeline);
    return;
    }

}
*/

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::AddObservations(vtkMRMLTransformNode* node)
{
  vtkEventBroker* broker = vtkEventBroker::GetInstance();
  if (!broker->GetObservationExist(node, vtkMRMLDisplayableNode::DisplayModifiedEvent, this->External, this->External->GetMRMLNodesCallbackCommand() ))
    {
    broker->AddObservation(node, vtkMRMLDisplayableNode::DisplayModifiedEvent, this->External, this->External->GetMRMLNodesCallbackCommand() );
    }
  if (!broker->GetObservationExist(node, vtkMRMLTransformableNode::TransformModifiedEvent, this->External, this->External->GetMRMLNodesCallbackCommand() ))
    {
    broker->AddObservation(node, vtkMRMLTransformableNode::TransformModifiedEvent, this->External, this->External->GetMRMLNodesCallbackCommand() );
    }
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
::AddDisplayObservations(vtkMRMLTransformDisplayNode* node)
{
  vtkEventBroker* broker = vtkEventBroker::GetInstance();
  if (!broker->GetObservationExist(node, vtkMRMLTransformDisplayNode::TransformUpdateEditorBoundsEvent, this->External, this->External->GetMRMLNodesCallbackCommand() ))
    {
    broker->AddObservation(node, vtkMRMLTransformDisplayNode::TransformUpdateEditorBoundsEvent, this->External, this->External->GetMRMLNodesCallbackCommand() );
    }
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::RemoveObservations(vtkMRMLTransformNode* node)
{
  vtkEventBroker* broker = vtkEventBroker::GetInstance();
  vtkEventBroker::ObservationVector observations;
  observations = broker->GetObservations(node, vtkMRMLDisplayableNode::DisplayModifiedEvent, this->External, this->External->GetMRMLNodesCallbackCommand() );
  observations = broker->GetObservations(node, vtkMRMLTransformableNode::TransformModifiedEvent, this->External, this->External->GetMRMLNodesCallbackCommand() );
  broker->RemoveObservations(observations);
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
::RemoveDisplayObservations(vtkMRMLTransformDisplayNode* node)
{
  vtkEventBroker* broker = vtkEventBroker::GetInstance();
  vtkEventBroker::ObservationVector observations;
  observations = broker->GetObservations(node, vtkMRMLTransformDisplayNode::TransformUpdateEditorBoundsEvent, this->External, this->External->GetMRMLNodesCallbackCommand() );
  broker->RemoveObservations(observations);
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::ClearDisplayableNodes()
{
  while(this->TransformToDisplayNodes.size() > 0)
    {
    this->RemoveTransformNode(this->TransformToDisplayNodes.begin()->first);
    }
}

//---------------------------------------------------------------------------
bool vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal::UseDisplayableNode(vtkMRMLTransformNode* node)
{
  bool use = node && node->IsA("vtkMRMLTransformNode");
  return use;
}

//---------------------------------------------------------------------------
// vtkMRMLLinearTransformsDisplayableManager3D methods

//---------------------------------------------------------------------------
vtkMRMLLinearTransformsDisplayableManager3D::vtkMRMLLinearTransformsDisplayableManager3D()
{
  this->Internal = new vtkInternal(this);
  this->DisableInteractorStyleEventsProcessing = 0;

  this->LastClickWorldCoordinates[0] = 0.0;
  this->LastClickWorldCoordinates[1] = 0.0;
  this->LastClickWorldCoordinates[2] = 0.0;
  this->LastClickWorldCoordinates[3] = 1.0;
}

//---------------------------------------------------------------------------
vtkMRMLLinearTransformsDisplayableManager3D::~vtkMRMLLinearTransformsDisplayableManager3D()
{
  delete this->Internal;
  this->Internal=nullptr;

  this->DisableInteractorStyleEventsProcessing = 0;
}

vtkSlicerLinearTransformWidget* vtkMRMLLinearTransformsDisplayableManager3D::FindClosestWidget(vtkMRMLInteractionEventData* callData, double& closestDistance2)
{
  vtkSlicerLinearTransformWidget* closestWidget = nullptr;
  closestDistance2 = VTK_DOUBLE_MAX;

  for (vtkInternal::DisplayNodeToWidgetIt widgetIterator = this->Internal->TransformDisplayNodesToWidgets.begin();
    widgetIterator != this->Internal->TransformDisplayNodesToWidgets.end(); ++widgetIterator)
  {
    vtkSlicerLinearTransformWidget* widget = widgetIterator->second;
    if (!widget)
    {
      continue;
    }
    double distance2FromWidget = VTK_DOUBLE_MAX;
    if (widget->CanProcessInteractionEvent(callData, distance2FromWidget))
    {
      if (!closestWidget || distance2FromWidget < closestDistance2)
      {
        closestDistance2 = distance2FromWidget;
        closestWidget = widget;
      }
    }
  }
  return closestWidget;
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::PrintSelf ( ostream& os, vtkIndent indent )
{
  this->Superclass::PrintSelf ( os, indent );
  os << indent << "vtkMRMLLinearTransformsDisplayableManager3D: "
     << this->GetClassName() << "\n";
}

bool vtkMRMLLinearTransformsDisplayableManager3D::CanProcessInteractionEvent(vtkMRMLInteractionEventData* eventData, double& closestDistance2)
{
  int eventid = eventData->GetType();
  if (eventid == vtkCommand::LeaveEvent && this->LastActiveWidget != nullptr)
  {
    if (this->LastActiveWidget->GetTransformDisplayNode() && this->LastActiveWidget->GetTransformDisplayNode()->HasActiveComponent())
    {
      // this widget has active component, therefore leave event is relevant
      closestDistance2 = 0.0;
      return this->LastActiveWidget;
    }
  }

  // Other interactions
  bool canProcess = (this->FindClosestWidget(eventData, closestDistance2) != nullptr);

  if (!canProcess && this->LastActiveWidget != nullptr
    && (eventid == vtkCommand::MouseMoveEvent || eventid == vtkCommand::Move3DEvent))
  {
    // interaction context (e.g. mouse) is moved away from the widget -> deactivate if it's the same context that activated it
    std::vector<std::string> contextsWithActiveComponents =
      this->LastActiveWidget->GetTransformDisplayNode()->GetActiveComponentInteractionContexts();
    if (std::find(contextsWithActiveComponents.begin(), contextsWithActiveComponents.end(), eventData->GetInteractionContextName())
      != contextsWithActiveComponents.end())
    {
      this->LastActiveWidget->Leave(eventData);
      this->LastActiveWidget = nullptr;
    }
  }

  return canProcess;
}

bool vtkMRMLLinearTransformsDisplayableManager3D::ProcessInteractionEvent(vtkMRMLInteractionEventData* eventData)
{
  if (this->GetDisableInteractorStyleEventsProcessing())
  {
    return false;
  }
  int eventid = eventData->GetType();

  if (eventid == vtkCommand::LeaveEvent)
  {
    if (this->LastActiveWidget != nullptr)
    {
      this->LastActiveWidget->Leave(eventData);
      this->LastActiveWidget = nullptr;
    }
  }

  // Find/create active widget. Using smart pointer instead of raw pointer to ensure activeWidget
  // object does not get fully deleted until we are done using it if the user deletes it as part
  // of an EndPlacementEvent

  double closestDistance2 = VTK_DOUBLE_MAX;
  vtkSmartPointer<vtkSlicerLinearTransformWidget> activeWidget = this->FindClosestWidget(eventData, closestDistance2);
  

  // Deactivate previous widget
  if (this->LastActiveWidget != nullptr && this->LastActiveWidget != activeWidget.GetPointer())
  {
    this->LastActiveWidget->Leave(eventData);
  }
  this->LastActiveWidget = activeWidget;
  if (!activeWidget)
  {
    // deactivate widget if we move far from it
    if (eventid == vtkCommand::MouseMoveEvent && this->LastActiveWidget != nullptr)
    {
      this->LastActiveWidget->Leave(eventData);
      this->LastActiveWidget = nullptr;
    }
    return false;
  }

  // Pass on the interaction event to the active widget
  return activeWidget->ProcessInteractionEvent(eventData);
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLSceneNodeAdded(vtkMRMLNode* node)
{
  if ( !node->IsA("vtkMRMLTransformNode") )
    {
    return;
    }

  // Escape if the scene a scene is being closed, imported or connected
  if (this->GetMRMLScene()->IsBatchProcessing())
    {
    this->SetUpdateFromMRMLRequested(true);
    return;
    }

  this->Internal->AddTransformNode(vtkMRMLTransformNode::SafeDownCast(node));
  this->RequestRender();
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLSceneNodeRemoved(vtkMRMLNode* node)
{
  if ( node
    && (!node->IsA("vtkMRMLTransformNode"))
    && (!node->IsA("vtkMRMLTransformDisplayNode")) )
    {
    return;
    }

  vtkMRMLTransformNode* transformNode = nullptr;
  vtkMRMLTransformDisplayNode* displayNode = nullptr;

  bool modified = false;
  if ( (transformNode = vtkMRMLTransformNode::SafeDownCast(node)) )
    {
    this->Internal->RemoveTransformNode(transformNode);
    modified = true;
    }
  else if ( (displayNode = vtkMRMLTransformDisplayNode::SafeDownCast(node)) )
    {
    this->Internal->RemoveDisplayNode(displayNode);
    modified = true;
    }
  if (modified)
    {
    this->RequestRender();
    }
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::ProcessMRMLNodesEvents(vtkObject* caller, unsigned long event, void* callData)
{
  vtkMRMLScene* scene = this->GetMRMLScene();

  if (scene == nullptr || scene->IsBatchProcessing())
    {
    return;
    }

  vtkMRMLTransformNode* displayableNode = vtkMRMLTransformNode::SafeDownCast(caller);
  vtkMRMLTransformDisplayNode* displayNode = vtkMRMLTransformDisplayNode::SafeDownCast(caller);
  vtkMRMLInteractionNode* interactionNode = vtkMRMLInteractionNode::SafeDownCast(caller);

  if ( displayableNode )
    {
    vtkMRMLNode* callDataNode = reinterpret_cast<vtkMRMLDisplayNode *> (callData);
    displayNode = vtkMRMLTransformDisplayNode::SafeDownCast(callDataNode);
   
    //new update
    bool renderRequested = false;

    for (int displayNodeIndex = 0; displayNodeIndex < displayableNode->GetNumberOfDisplayNodes(); displayNodeIndex++)
    {
      vtkMRMLTransformDisplayNode* displayNode_1 = vtkMRMLTransformDisplayNode::SafeDownCast(displayableNode->GetNthDisplayNode(displayNodeIndex));
      vtkSlicerLinearTransformWidget* widget = this->Internal->GetWidget(displayNode_1);
      if (!widget)
      {
        // if a new display node is added or display node view node IDs are changed then we may need to create a new widget
        this->Internal->AddDisplayNode(displayableNode,displayNode_1);
        widget = this->Internal->GetWidget(displayNode);
      }
      if (!widget)
      {
        continue;
      }
      widget->UpdateFromMRML(displayNode_1, event, callData);
      if (widget->GetNeedToRender())
      {
        renderRequested = true;
        widget->NeedToRenderOff();
      }
    }

    if (renderRequested)
    {
      this->RequestRender();
    }

    //old update
    /*if ( displayNode && (event == vtkMRMLDisplayableNode::DisplayModifiedEvent) )
      {
      this->Internal->UpdateDisplayNode(displayNode);
      this->RequestRender();
      }
    else if (event == vtkMRMLTransformableNode::TransformModifiedEvent)
      {
      this->Internal->UpdateDisplayableTransforms(displayableNode, false);
      this->RequestRender();
      }*/
    }
  /*else if ( displayNode )
    {
    displayableNode = vtkMRMLTransformNode::SafeDownCast(displayNode->GetDisplayableNode());
    if ( displayNode && event == vtkMRMLTransformDisplayNode::TransformUpdateEditorBoundsEvent)
      {
      this->Internal->UpdateDisplayableTransforms(displayableNode, true);
      this->RequestRender();
      }
    }*/
  else if (interactionNode)
  {
    if (event == vtkMRMLInteractionNode::InteractionModeChangedEvent)
    {
      // loop through all widgets and update the widget status
      for (vtkInternal::DisplayNodeToWidgetIt widgetIterator = this->Internal->TransformDisplayNodesToWidgets.begin();
        widgetIterator != this->Internal->TransformDisplayNodesToWidgets.end(); ++widgetIterator)
      {
        vtkSlicerLinearTransformWidget* widget = widgetIterator->second;
        if (!widget)
        {
          continue;
        }
        vtkMRMLInteractionEventData* eventData = reinterpret_cast<vtkMRMLInteractionEventData*>(callData);
        widget->Leave(eventData);
      }
    }
  }

  else
  {
    this->Superclass::ProcessMRMLNodesEvents(caller, event, callData);
  }
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::UpdateFromMRML()
{
  this->SetUpdateFromMRMLRequested(false);

  vtkMRMLScene* scene = this->GetMRMLScene();
  if (!scene)
    {
    vtkDebugMacro( "vtkMRMLLinearTransformsDisplayableManager3D->UpdateFromMRML: Scene is not set.");
    return;
    }
  this->Internal->ClearDisplayableNodes();

  vtkMRMLTransformNode* mNode = nullptr;
  std::vector<vtkMRMLNode *> mNodes;
  int nnodes = scene ? scene->GetNodesByClass("vtkMRMLTransformNode", mNodes) : 0;
  for (int i=0; i<nnodes; i++)
    {
    mNode  = vtkMRMLTransformNode::SafeDownCast(mNodes[i]);
    if (mNode && this->Internal->UseDisplayableNode(mNode))
      {
      this->Internal->AddTransformNode(mNode);
      }
    }
  this->RequestRender();
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::UnobserveMRMLScene()
{
  this->Internal->ClearDisplayableNodes();
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLSceneStartClose()
{
  this->Internal->ClearDisplayableNodes();
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLSceneEndClose()
{
  this->SetUpdateFromMRMLRequested(true);
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLSceneEndBatchProcess()
{
  this->SetUpdateFromMRMLRequested(true);
  this->RequestRender();
}

void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLDisplayableNodeModifiedEvent(vtkObject* caller)
{
  vtkDebugMacro("OnMRMLDisplayableNodeModifiedEvent");

  if (!caller)
  {
    vtkErrorMacro("OnMRMLDisplayableNodeModifiedEvent: Could not get caller.");
    return;
  }

  vtkMRMLSliceNode* sliceNode = vtkMRMLSliceNode::SafeDownCast(caller);
  if (sliceNode)
  {
    // the associated renderWindow is a 2D SliceView
    // this is the entry point for all events fired by one of the three sliceviews
    // (e.g. change slice number, zoom etc.)

    // we remember that this instance of the displayableManager deals with 2D
    // this is important for widget creation etc. and save the actual SliceNode
    // because during Slicer startup the SliceViews fire events, it will be always set correctly
    this->SliceNode = sliceNode;

    // now we call the handle for specific sliceNode actions
    this->OnMRMLSliceNodeModifiedEvent();

    // and exit
    return;
  }

  vtkMRMLViewNode* viewNode = vtkMRMLViewNode::SafeDownCast(caller);
  if (viewNode)
  {
    // the associated renderWindow is a 3D View
    vtkDebugMacro("OnMRMLDisplayableNodeModifiedEvent: This displayableManager handles a ThreeD view.");
    return;
  }
}

void vtkMRMLLinearTransformsDisplayableManager3D::OnMRMLSliceNodeModifiedEvent()
{
  bool renderRequested = false;

  // run through all transform nodes in the helper
  vtkInternal::DisplayNodeToWidgetIt it
    = this->Internal->TransformDisplayNodesToWidgets.begin();
  while (it != this->Internal->TransformDisplayNodesToWidgets.end())
  {
    // we loop through all widgets
    vtkSlicerLinearTransformWidget* widget = (it->second);
    widget->UpdateFromMRML(this->SliceNode, vtkCommand::ModifiedEvent);
    if (widget->GetNeedToRender())
    {
      renderRequested = true;
      widget->NeedToRenderOff();
    }
    ++it;
  }

  if (renderRequested)
  {
    this->RequestRender();
  }
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D::Create()
{
  Superclass::Create();
  this->SetUpdateFromMRMLRequested(true);
}

//---------------------------------------------------------------------------
void vtkMRMLLinearTransformsDisplayableManager3D
::ProcessWidgetsEvents(vtkObject* caller, unsigned long event, void* callData)
{
  Superclass::ProcessWidgetsEvents(caller, event, callData);

  /*vtkBoxWidget2* boxWidget = vtkBoxWidget2::SafeDownCast(caller);
  if (boxWidget)
    {
    this->Internal->UpdateNodeFromWidget(boxWidget);
    this->RequestRender();
    }*/
}

//---------------------------------------------------------------------------
/*vtkAbstractWidget* vtkMRMLLinearTransformsDisplayableManager3D
::GetWidget(vtkMRMLTransformDisplayNode* displayNode)
{
  vtkMRMLLinearTransformsDisplayableManager3D::vtkInternal
    ::PipelinesCacheType::iterator pipelineIter =
      this->Internal->DisplayPipelines.find(displayNode);
  if (pipelineIter != this->Internal->DisplayPipelines.end())
    {
    return pipelineIter->second->Widget;
    }
  return nullptr;
}*/
