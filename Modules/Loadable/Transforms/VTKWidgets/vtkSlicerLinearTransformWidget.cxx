/*=========================================================================

 Copyright (c) ProxSim ltd., Kwun Tong, Hong Kong. All Rights Reserved.

 See COPYRIGHT.txt
 or http://www.slicer.org/copyright/copyright.txt for details.

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 This file was originally developed by Davide Punzo, punzodavide@hotmail.it,
 and development was supported by ProxSim ltd.

=========================================================================*/

#include "vtkSlicerLinearTransformWidget.h"

#include "vtkMRMLInteractionEventData.h"
#include "vtkMRMLInteractionNode.h"
#include "vtkMRMLScene.h"
#include "vtkMRMLSliceCompositeNode.h"
#include "vtkMRMLSliceLogic.h"
#include "vtkSlicerLinearTransformWidgetRepresentation.h"

// VTK includes
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkEvent.h>
#include <vtkLine.h>
#include <vtkPlane.h>
#include <vtkPointPlacer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkObjectFactory.h>

// MRML includes
#include "vtkMRMLTransformNode.h"
#include <vtkMRMLApplicationLogic.h>

vtkStandardNewMacro(vtkSlicerLinearTransformWidget);

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidget::vtkSlicerLinearTransformWidget()
{
  this->LastEventPosition[0] = 0.0;
  this->LastEventPosition[1] = 0.0;
  this->StartEventOffsetPosition[0] = 0.0;
  this->StartEventOffsetPosition[1] = 0.0;

  // Manipulate
  this->SetEventTranslationClickAndDrag(WidgetStateOnWidget, vtkCommand::MiddleButtonPressEvent, vtkEvent::NoModifier,
    WidgetStateTranslate, WidgetEventTranslateStart, WidgetEventTranslateEnd);
  this->SetKeyboardEventTranslation(WidgetStateOnWidget, vtkEvent::ShiftModifier, 127, 1, "Delete", WidgetEventReset);

  this->SetEventTranslation(WidgetStateOnWidget, vtkMRMLInteractionEventData::LeftButtonClickEvent, vtkEvent::NoModifier, WidgetEventJumpCursor);
  this->SetEventTranslation(WidgetStateOnWidget, vtkCommand::LeftButtonDoubleClickEvent, vtkEvent::NoModifier, WidgetEventAction);

  unsigned int menuStates[] = { WidgetStateOnWidget, WidgetStateOnTranslationHandle, WidgetStateOnRotationHandle, WidgetStateOnScaleHandle };
  for (auto menuState : menuStates)
    {
    this->SetEventTranslation(menuState, vtkCommand::RightButtonPressEvent, vtkEvent::NoModifier, WidgetEventReserved);
    this->SetEventTranslation(menuState, vtkCommand::RightButtonReleaseEvent, vtkEvent::NoModifier, WidgetEventReserved);
    this->SetEventTranslation(menuState, vtkMRMLInteractionEventData::RightButtonClickEvent, vtkEvent::NoModifier, WidgetEventMenu);
    }

  // Update active component
  this->SetEventTranslation(WidgetStateIdle, vtkCommand::MouseMoveEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnWidget, vtkCommand::MouseMoveEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  // Allow AnyModifier when defining the markup position. This allows the markup preview to be continually updated, even
  // when using shift + mouse-move to change the slice positions.
  // We still do not allow shift+left click for placement however, so that the shift + left-click-and-drag interaction can
  // still be used to pan the slice.
  
  this->SetEventTranslation(WidgetStateIdle, vtkCommand::Move3DEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnWidget, vtkCommand::Move3DEvent, vtkEvent::NoModifier, WidgetEventMouseMove);

  // Handle interactions
  this->SetEventTranslationClickAndDrag(WidgetStateOnTranslationHandle, vtkCommand::LeftButtonPressEvent, vtkEvent::NoModifier,
    WidgetStateTranslate, WidgetEventTranslateStart, WidgetEventTranslateEnd);
  this->SetEventTranslation(WidgetStateOnTranslationHandle, vtkMRMLInteractionEventData::RightButtonClickEvent, vtkEvent::NoModifier, WidgetEventMenu);
  this->SetEventTranslation(WidgetStateOnTranslationHandle, vtkMRMLInteractionEventData::LeftButtonClickEvent, vtkEvent::NoModifier, WidgetEventJumpCursor);

  this->SetEventTranslationClickAndDrag(WidgetStateOnRotationHandle, vtkCommand::LeftButtonPressEvent, vtkEvent::NoModifier,
    WidgetStateRotate, WidgetEventRotateStart, WidgetEventRotateEnd);
  this->SetEventTranslation(WidgetStateOnRotationHandle, vtkMRMLInteractionEventData::RightButtonClickEvent, vtkEvent::NoModifier, WidgetEventMenu);
  this->SetEventTranslation(WidgetStateOnRotationHandle, vtkMRMLInteractionEventData::LeftButtonClickEvent, vtkEvent::NoModifier, WidgetEventJumpCursor);

  /*this->SetEventTranslationClickAndDrag(WidgetStateOnScaleHandle, vtkCommand::LeftButtonPressEvent, vtkEvent::NoModifier,
    WidgetStateScale, WidgetEventScaleStart, WidgetEventScaleEnd);*/
  this->SetEventTranslation(WidgetStateOnScaleHandle, vtkMRMLInteractionEventData::RightButtonClickEvent, vtkEvent::NoModifier, WidgetEventMenu);
  this->SetEventTranslation(WidgetStateOnScaleHandle, vtkMRMLInteractionEventData::LeftButtonClickEvent, vtkEvent::NoModifier, WidgetEventJumpCursor);


  // Update active interaction handle component
  this->SetEventTranslation(WidgetStateOnTranslationHandle, vtkCommand::MouseMoveEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnTranslationHandle, vtkCommand::Move3DEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnRotationHandle, vtkCommand::MouseMoveEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnRotationHandle, vtkCommand::Move3DEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnScaleHandle, vtkCommand::MouseMoveEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
  this->SetEventTranslation(WidgetStateOnScaleHandle, vtkCommand::Move3DEvent, vtkEvent::NoModifier, WidgetEventMouseMove);
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidget::~vtkSlicerLinearTransformWidget() = default;

//----------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessWidgetRotateStart(vtkMRMLInteractionEventData* eventData)
{
  if ((this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateOnWidget && this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateOnRotationHandle))
    {
    return false;
    }

  this->SetWidgetState(WidgetStateRotate);
  this->StartWidgetInteraction(eventData);
  return true;
}

//-------------------------------------------------------------------------
/*bool vtkSlicerLinearTransformWidget::ProcessWidgetScaleStart(vtkMRMLInteractionEventData* eventData)
{
  if ((this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateOnWidget && this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateOnScaleHandle))
    {
    return false;
    }

  this->SetWidgetState(WidgetStateScale);
  this->StartWidgetInteraction(eventData);
  return true;
}*/

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessWidgetTranslateStart(vtkMRMLInteractionEventData* eventData)
{
  if ((this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateOnWidget && this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateOnTranslationHandle))
    {
    return false;
    }

  this->SetWidgetState(WidgetStateTranslate);
  this->StartWidgetInteraction(eventData);
  return true;
}

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessMouseMove(vtkMRMLInteractionEventData* eventData)
{
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  vtkSlicerLinearTransformWidgetRepresentation* rep = this->GetTransformRepresentation();
  if (!rep || !transformNode || !eventData)
    {
    return false;
    }

  int state = this->WidgetState;

  if (state == WidgetStateIdle
    || state == WidgetStateOnWidget
    || state == WidgetStateOnTranslationHandle
    || state == WidgetStateOnRotationHandle
    || state == WidgetStateOnScaleHandle)
    {
    // update state
    int foundComponentType = vtkMRMLTransformDisplayNode::ComponentNone;
    int foundComponentIndex = -1;
    double closestDistance2 = 0.0;
    rep->CanInteract(eventData, foundComponentType, foundComponentIndex, closestDistance2);
    if (foundComponentType == vtkMRMLTransformDisplayNode::ComponentNone)
      {
      this->SetWidgetState(WidgetStateIdle);
      }
    else if (foundComponentType == vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
      {
      this->SetWidgetState(WidgetStateOnTranslationHandle);
      }
    else if (foundComponentType == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
      {
      this->SetWidgetState(WidgetStateOnRotationHandle);
      }
    else if (foundComponentType == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
      {
      this->SetWidgetState(WidgetStateOnScaleHandle);
      }
    else
      {
      this->SetWidgetState(WidgetStateOnWidget);
      }

    this->GetTransformDisplayNode()->SetActiveComponent(foundComponentType, foundComponentIndex, eventData->GetInteractionContextName());
    }
  else
    {
    // Process the motion
    // Based on the displacement vector (computed in display coordinates) and
    // the cursor state (which corresponds to which part of the widget has been
    // selected), the widget points are modified.
    // First construct a local coordinate system based on the display coordinates
    // of the widget.
    double eventPos[2]
    {
      static_cast<double>(eventData->GetDisplayPosition()[0]),
      static_cast<double>(eventData->GetDisplayPosition()[1]),
    };
    if (state == WidgetStateTranslate)
      {
      this->TranslateWidget(eventPos);
      }
    /*else if (state == WidgetStateScale)
      {
      this->ScaleWidget(eventPos);
      }*/
    else if (state == WidgetStateRotate)
      {
      this->RotateWidget(eventPos);
      }

    this->LastEventPosition[0] = eventPos[0];
    this->LastEventPosition[1] = eventPos[1];
    }

  return true;
}

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessEndMouseDrag(vtkMRMLInteractionEventData* eventData)
{
  if (!this->WidgetRep)
    {
    return false;
    }

  if ((this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateTranslate
    && this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateScale
    && this->WidgetState != vtkSlicerLinearTransformWidget::WidgetStateRotate
    ) || !this->WidgetRep)
    {
    return false;
    }

  int activeComponentType = this->GetActiveComponentType();
  if (activeComponentType == vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
    {
    this->SetWidgetState(WidgetStateOnTranslationHandle);
    }
  else if (activeComponentType == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
    {
    this->SetWidgetState(WidgetStateOnRotationHandle);
    }
  else if (activeComponentType == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
    {
    this->SetWidgetState(WidgetStateOnScaleHandle);
    }
  else
    {
    this->SetWidgetState(WidgetStateOnWidget);
    }

  this->EndWidgetInteraction();

  // only claim this as processed if the mouse was moved (this lets the event interpreted as button click)
  bool processedEvent = eventData->GetMouseMovedSinceButtonDown();
  return processedEvent;
}


//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessWidgetJumpCursor(vtkMRMLInteractionEventData* vtkNotUsed(eventData))
{
  /*if (this->WidgetState != WidgetStateOnWidget &&
    this->WidgetState != WidgetStateOnTranslationHandle &&
    this->WidgetState != WidgetStateOnRotationHandle &&
    this->WidgetState != WidgetStateOnScaleHandle)
    {
    return false;
    }

  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  vtkMRMLTransformDisplayNode* transformDisplayNode = this->GetTransformDisplayNode();
  if (!transformNode || !transformDisplayNode)
    {
    return false;
    }

  int componentIndex = transformDisplayNode->GetActiveComponentIndex();;
  int componentType = transformDisplayNode->GetActiveComponentType();

  // Use first active control point for jumping //TODO: Have an 'even more active' point concept
  if (componentType == vtkMRMLTransformDisplayNode::ComponentControlPoint)
    {
    std::vector<int> activeControlPointIndices;
    transformDisplayNode->GetActiveControlPoints(activeControlPointIndices);

    if (!activeControlPointIndices.empty())
      {
      componentIndex = activeControlPointIndices[0];
      }
    if (componentIndex < 0 || componentIndex >= transformNode->GetNumberOfControlPoints())
      {
      return false;
      }
    }

  transformNode->GetScene()->SaveStateForUndo();

  vtkNew<vtkMRMLInteractionEventData> jumpToPointEventData;
  jumpToPointEventData->SetType(vtkMRMLTransformDisplayNode::JumpToPointEvent);
  jumpToPointEventData->SetComponentType(componentType);
  jumpToPointEventData->SetComponentIndex(componentIndex);
  jumpToPointEventData->SetViewNode(this->WidgetRep->GetViewNode());

  if (componentType == vtkMRMLTransformDisplayNode::ComponentRotationHandle
    || componentType == vtkMRMLTransformDisplayNode::ComponentTranslationHandle
    || componentType == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
    {
    // For interaction handle, send the position of the handle as well.
    // The position of the handle may be different in each view, so we need to get the position from the representation.
    vtkSlicerLinearTransformWidgetRepresentation* rep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
    if (rep)
      {
      double position_World[3] = { 0.0, 0.0, 0.0 };
      rep->GetInteractionHandlePositionWorld(componentType, transformDisplayNode->GetActiveComponentIndex(), position_World);
      jumpToPointEventData->SetWorldPosition(position_World);
      }
    }

  transformDisplayNode->InvokeEvent(vtkMRMLTransformDisplayNode::JumpToPointEvent, jumpToPointEventData);
  return true;*/
  return true;
}

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ConvertDisplayPositionToWorld(const int displayPos[2],
  double worldPos[3], double worldOrientationMatrix[9], double* refWorldPos/*=nullptr*/)
{
  
  vtkSlicerLinearTransformWidgetRepresentation* rep3d = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
  double doubleDisplayPos[3] = { static_cast<double>(displayPos[0]), static_cast<double>(displayPos[1]), 0.0 };
  if (rep3d)
    {
    // 3D view
    bool preferPickOnSurface = true;
    /*if (refWorldPos != nullptr)
      {
      // If reference position is provided then we may use that instead of picking on visible surface.
      vtkMRMLTransformDisplayNode* transformDisplayNode = this->GetTransformDisplayNode();
      if (transformDisplayNode)
        {
        preferPickOnSurface = (transformDisplayNode->GetSnapMode() == vtkMRMLTransformDisplayNode::SnapModeToVisibleSurface);
        }
      }*/
    if (preferPickOnSurface)
      {
      // SnapModeToVisibleSurface
      // Try to pick on surface and pick on camera plane if nothing is found.
      if (rep3d->AccuratePick(displayPos[0], displayPos[1], worldPos))
        {
        return true;
        }
      if (refWorldPos)
        {
        // Reference position is available (most likely, moving the point).
        return (rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
          doubleDisplayPos, refWorldPos, worldPos, worldOrientationMatrix));
        }
      }
    else
      {
      // SnapModeUnconstrained
      // Move the point relative to reference position, not restricted to surfaces if possible.
      if (refWorldPos)
        {
        // Reference position is available (most likely, moving the point).
        return (rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
          doubleDisplayPos, refWorldPos, worldPos, worldOrientationMatrix));
        }
      else
        {
        // Reference position is unavailable (e.g., not moving of an existing point but first placement)
        // Even if the constraining on the surface is no preferred, it is still better to
        // place it on a visible surface in 3D views rather on the .
        if (rep3d->AccuratePick(displayPos[0], displayPos[1], worldPos))
          {
          return true;
          }
        }
      }
    // Last resort: place a point on the camera plane
    // (no reference position is available and no surface is visible there)
    return (rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      doubleDisplayPos, worldPos, worldOrientationMatrix));
    }
  return false;
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidget::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

void vtkSlicerLinearTransformWidget::CreateDefaultRepresentation(vtkMRMLTransformDisplayNode* transformDisplayNode,
  vtkMRMLAbstractViewNode* viewNode, vtkRenderer* renderer)
{
  vtkSmartPointer<vtkSlicerLinearTransformWidgetRepresentation> rep = nullptr;
  
  rep = vtkSmartPointer<vtkSlicerLinearTransformWidgetRepresentation>::New();
  
  this->SetRenderer(renderer);
  this->SetRepresentation(rep);
  rep->SetViewNode(viewNode);
  rep->SetTransformDisplayNode(transformDisplayNode);
  rep->UpdateFromMRML(nullptr, 0); // full update
}

vtkSlicerLinearTransformWidget* vtkSlicerLinearTransformWidget::CreateInstance() const
{
  vtkObject* ret = vtkObjectFactory::CreateInstance("vtkSlicerLinearTransformWidget");
  if (ret)
  {
    return static_cast<vtkSlicerLinearTransformWidget*>(ret);
  }
  vtkSlicerLinearTransformWidget* result = new vtkSlicerLinearTransformWidget;
  result->InitializeObjectBase();
  return result;
}

//-----------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::CanProcessInteractionEvent(vtkMRMLInteractionEventData* eventData, double &distance2)
{
  unsigned long widgetEvent = this->TranslateInteractionEventToWidgetEvent(eventData);
  if (widgetEvent == WidgetEventNone)
    {
    // If this event is not recognized then give a chance to process it as a click event.
    return this->CanProcessButtonClickEvent(eventData, distance2);
    }
  vtkSlicerLinearTransformWidgetRepresentation* rep = this->GetTransformRepresentation();
  if (!rep)
    {
    return false;
    }

  // If we are placing transform or dragging the mouse then we interact everywhere
  if (this->WidgetState == WidgetStateTranslate
    || this->WidgetState == WidgetStateRotate
    || this->WidgetState == WidgetStateScale)
    {
    distance2 = 0.0;
    return true;
    }

  int foundComponentType = vtkMRMLTransformDisplayNode::ComponentNone;
  int foundComponentIndex = -1;
  double closestDistance2 = 0.0;
  rep->CanInteract(eventData, foundComponentType, foundComponentIndex, closestDistance2);
  if (foundComponentType == vtkMRMLTransformDisplayNode::ComponentNone)
    {
    return false;
    }
  distance2 = closestDistance2;
  return true;
}

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessWidgetMenu(vtkMRMLInteractionEventData* eventData)
{
  if (this->WidgetState != WidgetStateOnWidget &&
       this->WidgetState != WidgetStateOnTranslationHandle &&
       this->WidgetState != WidgetStateOnRotationHandle &&
       this->WidgetState != WidgetStateOnScaleHandle)
    {
    return false;
    }

  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  vtkMRMLTransformDisplayNode* transformDisplayNode = this->GetTransformDisplayNode();
  if (!transformNode || !transformDisplayNode)
    {
    return false;
    }

  vtkNew<vtkMRMLInteractionEventData> menuEventData;
  menuEventData->SetType(vtkMRMLDisplayNode::MenuEvent);
  menuEventData->SetComponentType(transformDisplayNode->GetActiveComponentType()); //TODO: This will always pass the active component for the mouse
  menuEventData->SetComponentIndex(transformDisplayNode->GetActiveComponentIndex());
  menuEventData->SetViewNode(this->WidgetRep->GetViewNode());

  // Copy display position
  if (eventData->IsDisplayPositionValid())
    {
    menuEventData->SetDisplayPosition(eventData->GetDisplayPosition());
    }

  // Copy/compute world position
  double worldPos[3] = { 0.0 };
  if (eventData->IsWorldPositionValid())
    {
    eventData->GetWorldPosition(worldPos);
    menuEventData->SetWorldPosition(worldPos, eventData->IsWorldPositionAccurate());
    }
  else if (eventData->IsDisplayPositionValid())
    {
    double worldOrientationMatrix[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    int displayPos[2] = { 0 };
    eventData->GetDisplayPosition(displayPos);
    if (this->ConvertDisplayPositionToWorld(displayPos, worldPos, worldOrientationMatrix))
      {
      menuEventData->SetWorldPosition(worldPos);
      }
    }

  transformDisplayNode->InvokeEvent(vtkMRMLDisplayNode::MenuEvent, menuEventData);
  return true;
}

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::ProcessWidgetAction(vtkMRMLInteractionEventData* eventData)
{
  /*if (this->WidgetState != WidgetStateOnWidget)
    {
    return false;
    }
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  vtkMRMLTransformDisplayNode* transformDisplayNode = this->GetTransformDisplayNode();
  if (!transformNode || !transformDisplayNode)
    {
    return false;
    }
  // Use first active control point for jumping //TODO: Have an 'even more active' point concept
  std::vector<int> activeControlPointIndices;
  transformDisplayNode->GetActiveControlPoints(activeControlPointIndices);
  int controlPointIndex = -1;
  if (!activeControlPointIndices.empty())
    {
    controlPointIndex = activeControlPointIndices[0];
    }
  if (controlPointIndex < 0 || controlPointIndex >= transformNode->GetNumberOfControlPoints())
    {
    return false;
    }
  transformNode->GetScene()->SaveStateForUndo();

  // Convert widget action to display node event
  unsigned long displayNodeEvent = vtkMRMLTransformDisplayNode::event;
  unsigned long widgetEvent = this->TranslateInteractionEventToWidgetEvent(eventData);
  switch (widgetEvent)
  {
  case WidgetEventCustomAction1: displayNodeEvent = vtkMRMLTransformDisplayNode::CustomActionEvent1; break;
  case WidgetEventCustomAction2: displayNodeEvent = vtkMRMLTransformDisplayNode::CustomActionEvent2; break;
  case WidgetEventCustomAction3: displayNodeEvent = vtkMRMLTransformDisplayNode::CustomActionEvent3; break;
  case WidgetEventCustomAction4: displayNodeEvent = vtkMRMLTransformDisplayNode::CustomActionEvent4; break;
  case WidgetEventCustomAction5: displayNodeEvent = vtkMRMLTransformDisplayNode::CustomActionEvent5; break;
  case WidgetEventCustomAction6: displayNodeEvent = vtkMRMLTransformDisplayNode::CustomActionEvent6; break;
  case WidgetEventAction:
  default:
    displayNodeEvent = vtkMRMLTransformDisplayNode::ActionEvent;
    break;
  }

  vtkNew<vtkMRMLInteractionEventData> actionEventData;
  actionEventData->SetType(displayNodeEvent);
  actionEventData->SetComponentType(vtkMRMLTransformDisplayNode::ComponentControlPoint);
  actionEventData->SetComponentIndex(controlPointIndex);
  actionEventData->SetViewNode(this->WidgetRep->GetViewNode());
  transformDisplayNode->InvokeEvent(displayNodeEvent, actionEventData);*/
  return true;
}

//-----------------------------------------------------------------------------

bool vtkSlicerLinearTransformWidget::ProcessInteractionEvent(vtkMRMLInteractionEventData* eventData)
{
  unsigned long widgetEvent = this->TranslateInteractionEventToWidgetEvent(eventData);

  if (this->ApplicationLogic)
    {
    this->ApplicationLogic->PauseRender();
    }


  bool processedEvent = false;
  switch (widgetEvent)
    {
    case WidgetEventMouseMove:
      processedEvent = ProcessMouseMove(eventData);
      break;
    case WidgetEventMenu:
      processedEvent = ProcessWidgetMenu(eventData);
      break;
    case WidgetEventAction:
    case WidgetEventCustomAction1:
    case WidgetEventCustomAction2:
    case WidgetEventCustomAction3:
      processedEvent = ProcessWidgetAction(eventData);
      break;
    case WidgetEventTranslateStart:
      processedEvent = ProcessWidgetTranslateStart(eventData);
      break;
    case WidgetEventTranslateEnd:
      processedEvent = ProcessEndMouseDrag(eventData);
      break;
    case WidgetEventRotateStart:
      processedEvent = ProcessWidgetRotateStart(eventData);
      break;
    case WidgetEventRotateEnd:
      processedEvent = ProcessEndMouseDrag(eventData);
      break;
    /*case WidgetEventScaleStart:
      processedEvent = ProcessWidgetScaleStart(eventData);
      break;*/
    case WidgetEventScaleEnd:
      processedEvent = ProcessEndMouseDrag(eventData);
      break;
    case WidgetEventJumpCursor:
      processedEvent = ProcessWidgetJumpCursor(eventData);
      break;
    }

  if (!processedEvent)
    {
    processedEvent = this->ProcessButtonClickEvent(eventData);
    }

  if (this->ApplicationLogic)
    {
    this->ApplicationLogic->ResumeRender();
    }

  return processedEvent;
}

//-----------------------------------------------------------------------------
void vtkSlicerLinearTransformWidget::Leave(vtkMRMLInteractionEventData* eventData)
{
  // Ensure that EndInteractionEvent is invoked, even if interrupted by an unexpected event
  if (this->WidgetState == vtkSlicerLinearTransformWidget::WidgetStateTranslate
    || this->WidgetState == vtkSlicerLinearTransformWidget::WidgetStateScale
    || this->WidgetState == vtkSlicerLinearTransformWidget::WidgetStateRotate)
    {
    this->EndWidgetInteraction();
    }

  vtkMRMLTransformDisplayNode* transformDisplayNode = this->GetTransformDisplayNode();
  if (transformDisplayNode)
    {
    std::string interactionContext("");
    if (eventData)
      {
      interactionContext = eventData->GetInteractionContextName();
      }
    transformDisplayNode->SetActiveComponent(vtkMRMLTransformDisplayNode::ComponentNone, -1, interactionContext);
    }
  Superclass::Leave(eventData);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidget::StartWidgetInteraction(vtkMRMLInteractionEventData* eventData)
{
  vtkSlicerLinearTransformWidgetRepresentation* rep = this->GetTransformRepresentation();
  if (!rep)
    {
    return;
    }
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  vtkMRMLTransformDisplayNode* transformDisplayNode = this->GetTransformDisplayNode();
  if (!transformNode || !transformDisplayNode)
    {
    return;
    }

  transformNode->GetScene()->SaveStateForUndo();

  double startEventPos[2]
    {
    static_cast<double>(eventData->GetDisplayPosition()[0]),
    static_cast<double>(eventData->GetDisplayPosition()[1])
    };

  // save the cursor position
  this->LastEventPosition[0] = startEventPos[0];
  this->LastEventPosition[1] = startEventPos[1];
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidget::EndWidgetInteraction()
{
  /*vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode)
    {
    return;
    }
  transformNode->InvokeCustomModifiedEvent(vtkMRMLTransformNode::PointEndInteractionEvent);
  // AddControlPoint will fire modified events anyway, so we temporarily disable events
  // to add a new point with a minimum number of events.
  bool wasDisabled = transformNode->GetDisableModifiedEvent();
  transformNode->DisableModifiedEventOn();
  transformNode->SetAttribute("Transform.MovingInSliceView", "");
  transformNode->SetAttribute("Transform.MovingMarkupIndex", "");
  transformNode->SetDisableModifiedEvent(wasDisabled);*/
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidget::TranslateWidget(double eventPos[2])
{
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode)
    {
    return;
    }

  double lastEventPos_World[3] = { 0.0 };
  double eventPos_World[3] = { 0.0 };
  double orientation_World[9] = { 0.0 };

  vtkSlicerLinearTransformWidgetRepresentation* rep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
  
  if (rep)
    {
    // 3D view
    if (!rep->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      this->LastEventPosition, lastEventPos_World, orientation_World))
      {
      return;
      }
    if (!rep->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      eventPos, lastEventPos_World, eventPos_World, orientation_World))
      {
      return;
      }
    }

  double translationVector_World[3];
  translationVector_World[0] = eventPos_World[0] - lastEventPos_World[0];
  translationVector_World[1] = eventPos_World[1] - lastEventPos_World[1];
  translationVector_World[2] = eventPos_World[2] - lastEventPos_World[2];
  int type = this->GetActiveComponentType();
  if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle && this->GetTransformDisplayNode())
    {
    int index = this->GetTransformDisplayNode()->GetActiveComponentIndex();

    double translationAxis_World[3] = { 0 };
    rep->GetInteractionHandleAxisWorld(type, index, translationAxis_World);

    // Only perform constrained translation if the length of the axis is non-zero.
    if (vtkMath::Norm(translationAxis_World) > 0)
      {
      double lastEventPositionOnAxis_World[3] = { 0.0, 0.0, 0.0 };
      this->GetClosestPointOnInteractionAxis(
        vtkMRMLTransformDisplayNode::ComponentTranslationHandle, index, this->LastEventPosition, lastEventPositionOnAxis_World);

      double eventPositionOnAxis_World[3] = { 0.0, 0.0, 0.0 };
      this->GetClosestPointOnInteractionAxis(
        vtkMRMLTransformDisplayNode::ComponentTranslationHandle, index, eventPos, eventPositionOnAxis_World);

      vtkMath::Subtract(eventPositionOnAxis_World, lastEventPositionOnAxis_World, translationVector_World);
      double distance = vtkMath::Norm(translationVector_World);
      if (vtkMath::Dot(translationVector_World, translationAxis_World) < 0)
        {
        distance *= -1.0;
        }
      translationVector_World[0] = distance * translationAxis_World[0];
      translationVector_World[1] = distance * translationAxis_World[1];
      translationVector_World[2] = distance * translationAxis_World[2];
      }
    }

  vtkNew<vtkTransform> T_WorldToNode;
  T_WorldToNode->PostMultiply();

  vtkNew<vtkMatrix4x4> matrix;
  transformNode->GetMatrixTransformToWorld(matrix);
  T_WorldToNode->SetMatrix(matrix);
  T_WorldToNode->Translate(translationVector_World);

  transformNode->SetMatrixTransformToWorld(T_WorldToNode->GetMatrix());
}

//----------------------------------------------------------------------
/*void vtkSlicerLinearTransformWidget::ScaleWidget(double eventPos[2])
{
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode)
    {
    return;
    }

  double center[3] = { 0. };
  double ref[3] = { 0. };
  double worldPos[3], worldOrient[9];

  vtkSlicerLinearTransformWidgetRepresentation* rep3d = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
  if (rep3d)
    {
    double displayPos[2] = { 0. };
    displayPos[0] = this->LastEventPosition[0];
    displayPos[1] = this->LastEventPosition[1];
    if (rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      displayPos, ref, worldPos,
      worldOrient))
      {
      for (int i = 0; i < 3; i++)
        {
        ref[i] = worldPos[i];
        }
      }
    else
      {
      return;
      }
    displayPos[0] = eventPos[0];
    displayPos[1] = eventPos[1];

    if (!rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      displayPos, ref, worldPos,
      worldOrient))
      {
      return;
      }

    rep3d->GetTransformationReferencePoint(center);
    }

  double r2 = vtkMath::Distance2BetweenPoints(ref, center);
  double d2 = vtkMath::Distance2BetweenPoints(worldPos, center);
  if (d2 < 0.0000001)
    {
    return;
    }

  double ratio = sqrt(d2 / r2);

  
  //todo apply ratio
}*/

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidget::RotateWidget(double eventPos[2])
{
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode)
    {
    return;
    }

  double eventPos_World[3] = { 0. };
  double lastEventPos_World[3] = { 0. };
  double orientation_World[9] = { 0. };
  double eventPos_Display[2] = { 0. };

  vtkSlicerLinearTransformWidgetRepresentation* rep3d = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
  if (rep3d)
    {
    if (rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      this->LastEventPosition, lastEventPos_World, orientation_World))
      {
      for (int i = 0; i < 3; i++)
        {
        eventPos_World[i] = lastEventPos_World[i];
        }
      }
    else
      {
      return;
      }
    eventPos_Display[0] = eventPos[0];
    eventPos_Display[1] = eventPos[1];
    if (!rep3d->GetPointPlacer()->ComputeWorldPosition(this->Renderer,
      eventPos_Display, eventPos_World, eventPos_World, orientation_World))
      {
      return;
      }
    }

  double origin_World[3] = { 0.0 };
  rep3d->GetInteractionHandleOriginWorld(origin_World);

  double epsilon = 1e-5;
  double d2 = vtkMath::Distance2BetweenPoints(eventPos_World, origin_World);
  if (d2 < epsilon)
    {
    return;
    }

  for (int i = 0; i < 3; i++)
    {
    lastEventPos_World[i] -= origin_World[i];
    eventPos_World[i] -= origin_World[i];
    }

  double angle = vtkMath::DegreesFromRadians(
    vtkMath::AngleBetweenVectors(lastEventPos_World, eventPos_World));
  double rotationNormal_World[3] = { 0.0 };
  vtkMath::Cross(lastEventPos_World, eventPos_World, rotationNormal_World);
  double rotationAxis_World[3] = { 0.0, 1.0, 0.0 };
  int type = this->GetActiveComponentType();
  if (type == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
    {
    int index = this->GetTransformDisplayNode()->GetActiveComponentIndex();
    double eventPositionOnAxisPlane_World[3] = { 0.0, 0.0, 0.0 };
    if (!this->GetIntersectionOnAxisPlane(type, index, eventPos, eventPositionOnAxisPlane_World))
      {
      vtkWarningMacro("RotateWidget: Could not calculate intended orientation");
      return;
      }

    rep3d->GetInteractionHandleAxisWorld(type, index, rotationAxis_World); // Axis of rotation
    double origin_World[3] = { 0.0, 0.0, 0.0 };
    rep3d->GetInteractionHandleOriginWorld(origin_World);

    double lastEventPositionOnAxisPlane_World[3] = { 0.0, 0.0, 0.0 };
    if (!this->GetIntersectionOnAxisPlane(
      vtkMRMLTransformDisplayNode::ComponentRotationHandle, index, this->LastEventPosition,lastEventPositionOnAxisPlane_World))
      {
      vtkWarningMacro("RotateWidget: Could not calculate previous orientation");
      return;
      }

    double rotationHandleVector_World[3] = { 0.0, 0.0, 0.0 };
    vtkMath::Subtract(lastEventPositionOnAxisPlane_World, origin_World, rotationHandleVector_World);

    double destinationVector_World[3] = { 0.0, 0.0, 0.0 };
    vtkMath::Subtract(eventPositionOnAxisPlane_World, origin_World, destinationVector_World);

    angle = vtkMath::DegreesFromRadians(vtkMath::AngleBetweenVectors(rotationHandleVector_World, destinationVector_World));
    vtkMath::Cross(rotationHandleVector_World, destinationVector_World, rotationNormal_World);
    }
  else
    {
    rotationAxis_World[0] = rotationNormal_World[0];
    rotationAxis_World[1] = rotationNormal_World[1];
    rotationAxis_World[2] = rotationNormal_World[2];
    }

  if (vtkMath::Dot(rotationNormal_World, rotationAxis_World) < 0.0)
    {
    angle *= -1.0;
    }

  MRMLNodeModifyBlocker blocker(transformNode);

  vtkNew<vtkMatrix4x4> t;
  transformNode->GetMatrixTransformToWorld(t);
 
  vtkNew<vtkTransform> T_WorldToNode;
  T_WorldToNode->PostMultiply();
  T_WorldToNode->Concatenate(t);
  T_WorldToNode->Translate(-origin_World[0], -origin_World[1], -origin_World[2]);
  T_WorldToNode->RotateWXYZ(angle, rotationAxis_World);
  T_WorldToNode->Translate(origin_World[0], origin_World[1], origin_World[2]);

  transformNode->SetMatrixTransformToWorld(T_WorldToNode->GetMatrix());
}

//----------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::GetIntersectionOnAxisPlane(int type, int index, const double input_Display[2], double outputIntersection_World[3])
{
  vtkSlicerLinearTransformWidgetRepresentation* rep3d = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);

  double rotationAxis[3] = { 0 };
  rep3d->GetInteractionHandleAxisWorld(type, index, rotationAxis); // Axis of rotation
  double origin[3] = { 0, 0, 0 };
  rep3d->GetInteractionHandleOriginWorld(origin);

  vtkNew<vtkPlane> axisPlaneWorld;
  axisPlaneWorld->SetNormal(rotationAxis);
  axisPlaneWorld->SetOrigin(origin);

  double inputPoint0_World[3] = { 0.0, 0.0, 0.0 };
  double inputPoint1_World[3] = { 0.0, 0.0, 1.0 };
  double projectionVector_World[3] = { 0 };
  if (rep3d)
    {
    vtkRenderer* renderer = rep3d->GetRenderer();
    vtkCamera* camera = renderer->GetActiveCamera();

    // Focal point position
    double cameraFP_World[4] = { 0 };
    camera->GetFocalPoint(cameraFP_World);

    renderer->SetWorldPoint(cameraFP_World[0], cameraFP_World[1], cameraFP_World[2], cameraFP_World[3]);
    renderer->WorldToDisplay();
    double* cameraFP_Display = renderer->GetDisplayPoint();
    double selectionZ_Display = cameraFP_Display[2];

    renderer->SetDisplayPoint(input_Display[0], input_Display[1], selectionZ_Display);
    renderer->DisplayToWorld();
    double* input_World = renderer->GetWorldPoint();
    if (input_World[3] == 0.0)
      {
      vtkWarningMacro("Bad homogeneous coordinates");
      return false;
      }
    double pickPosition_World[3] = { 0.0 };
    for (int i = 0; i < 3; i++)
      {
      pickPosition_World[i] = input_World[i] / input_World[3];
      }
    if (camera->GetParallelProjection())
      {
      camera->GetDirectionOfProjection(projectionVector_World);
      for (int i = 0; i < 3; i++)
        {
        inputPoint0_World[i] = pickPosition_World[i];
        }
      }
    else
      {
      // Camera position
      double cameraPosition_World[4] = { 0.0 };
      camera->GetPosition(cameraPosition_World);

      //  Compute the ray endpoints.  The ray is along the line running from
      //  the camera position to the selection point, starting where this line
      //  intersects the front clipping plane, and terminating where this
      //  line intersects the back clipping plane.
      for (int i = 0; i < 3; i++)
        {
        projectionVector_World[i] = pickPosition_World[i] - cameraPosition_World[i];
        inputPoint0_World[i] = cameraPosition_World[i];
        }
      }
    vtkMath::Add(inputPoint0_World, projectionVector_World, inputPoint1_World);
    }

  double t = 0.0; // not used
  axisPlaneWorld->IntersectWithLine(inputPoint0_World, inputPoint1_World, t, outputIntersection_World);
  return true;
}

//----------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::GetClosestPointOnInteractionAxis(int type, int index, const double input_Display[2], double outputClosestPoint_World[3])
{
  vtkSlicerLinearTransformWidgetRepresentation* rep3d = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);

  double translationAxis_World[3] = { 0 };
  rep3d->GetInteractionHandleAxisWorld(type, index, translationAxis_World); // Axis of rotation
  double origin_World[3] = { 0, 0, 0 };
  rep3d->GetInteractionHandleOriginWorld(origin_World);

  double inputPoint0_World[3] = { 0.0, 0.0, 0.0 };
  double inputPoint1_World[3] = { 0.0, 0.0, 1.0 };
  if (rep3d)
    {
    vtkRenderer* renderer = rep3d->GetRenderer();
    vtkCamera* camera = renderer->GetActiveCamera();

    // Focal point position
    double cameraFP_World[4] = { 0 };
    camera->GetFocalPoint(cameraFP_World);

    renderer->SetWorldPoint(cameraFP_World[0], cameraFP_World[1], cameraFP_World[2], cameraFP_World[3]);
    renderer->WorldToDisplay();
    double* displayCoords = renderer->GetDisplayPoint();
    double selectionZ = displayCoords[2];

    renderer->SetDisplayPoint(input_Display[0], input_Display[1], selectionZ);
    renderer->DisplayToWorld();
    double* input_World = renderer->GetWorldPoint();
    if (input_World[3] == 0.0)
      {
      vtkWarningMacro("Bad homogeneous coordinates");
      return false;
      }
    double pickPosition_World[3] = { 0 };
    for (int i = 0; i < 3; i++)
      {
      pickPosition_World[i] = input_World[i] / input_World[3];
      }

    double projectionVector_World[3] = { 0 };
    if (camera->GetParallelProjection())
      {
      camera->GetDirectionOfProjection(projectionVector_World);
      for (int i = 0; i < 3; i++)
        {
        inputPoint0_World[i] = pickPosition_World[i];
        }
      }
    else
      {
      // Camera position
      double cameraPosition_World[4] = { 0 };
      camera->GetPosition(cameraPosition_World);

      //  Compute the ray endpoints.  The ray is along the line running from
      //  the camera position to the selection point, starting where this line
      //  intersects the front clipping plane, and terminating where this
      //  line intersects the back clipping plane.
      for (int i = 0; i < 3; i++)
        {
        inputPoint0_World[i] = cameraPosition_World[i];
        projectionVector_World[i] = pickPosition_World[i] - cameraPosition_World[i];
        }
      }
    vtkMath::Add(inputPoint0_World, projectionVector_World, inputPoint1_World);
    }
  
  double t1; // not used
  double t2; // not used
  double closestPointNotUsed[3] = { 0 };
  double translationVectorPoint[3] = { 0 };
  vtkMath::Add(origin_World, translationAxis_World, translationVectorPoint);
  vtkLine::DistanceBetweenLines(origin_World, translationVectorPoint,
    inputPoint0_World, inputPoint1_World, outputClosestPoint_World, closestPointNotUsed, t1, t2);
  return true;
}

//----------------------------------------------------------------------
vtkMRMLTransformNode* vtkSlicerLinearTransformWidget::GetTransformNode()
{
  vtkSlicerLinearTransformWidgetRepresentation* widgetRep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
  if (!widgetRep)
    {
    return nullptr;
    }
  return widgetRep->GetTransformNode();
}

//----------------------------------------------------------------------
vtkMRMLTransformDisplayNode* vtkSlicerLinearTransformWidget::GetTransformDisplayNode()
{
  vtkSlicerLinearTransformWidgetRepresentation* widgetRep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
  if (!widgetRep)
    {
    return nullptr;
    }
  return widgetRep->GetTransformDisplayNode();
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation* vtkSlicerLinearTransformWidget::GetTransformRepresentation()
{
  return vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->WidgetRep);
}

//-------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidget::GetInteractive()
{
  switch (this->WidgetState)
    {
    case WidgetStateTranslate:
    case WidgetStateScale:
    case WidgetStateRotate:
      return true;
    default:
      return false;
    }
}

//-------------------------------------------------------------------------
int vtkSlicerLinearTransformWidget::GetMouseCursor()
{
  if (this->WidgetState == WidgetStateIdle ||
    this->WidgetState == WidgetStateOnNothing) // default cursor shape is the "place" cursor
    {
    return VTK_CURSOR_DEFAULT;
    }
  else
    {
    return VTK_CURSOR_HAND;
    }
}

//---------------------------------------------------------------------------
int vtkSlicerLinearTransformWidget::GetActiveComponentType()
{
  vtkMRMLTransformDisplayNode* displayNode = this->GetTransformDisplayNode();
  if (!displayNode)
    {
    return vtkMRMLTransformDisplayNode::ComponentNone;
    }
  return displayNode->GetActiveComponentType();
}

//---------------------------------------------------------------------------
int vtkSlicerLinearTransformWidget::GetActiveComponentIndex()
{
  vtkMRMLTransformDisplayNode* displayNode = this->GetTransformDisplayNode();
  if (!displayNode)
    {
    return -1;
    }
  return displayNode->GetActiveComponentIndex();
}

//-----------------------------------------------------------------------------
vtkMRMLSelectionNode* vtkSlicerLinearTransformWidget::selectionNode()
{
  return vtkMRMLSelectionNode::SafeDownCast(
    this->GetTransformNode()->GetScene()->GetNodeByID("vtkMRMLSelectionNodeSingleton"));

}