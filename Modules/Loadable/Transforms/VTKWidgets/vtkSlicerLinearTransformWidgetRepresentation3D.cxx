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

// VTK includes
#include "vtkSlicerLinearTransformWidgetRepresentation3D.h"

#include "vtkCamera.h"
#include "vtkCellPicker.h"
#include "vtkGlyph3DMapper.h"
#include "vtkLine.h"
#include "vtkMarkupsGlyphSource2D.h"
#include "vtkMath.h"
#include "vtkPointData.h"
#include "vtkPointSetToLabelHierarchy.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkTextActor.h"

// MRML includes
#include <vtkMRMLFolderDisplayNode.h>
#include <vtkMRMLInteractionEventData.h>
#include <vtkMRMLViewNode.h>

vtkStandardNewMacro(vtkSlicerLinearTransformWidgetRepresentation3D);

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation3D::vtkSlicerLinearTransformWidgetRepresentation3D()
{
  // this->TextActor->SetTextProperty(this->GetControlPointsPipeline(Unselected)->TextProperty);
  this->TextActorPositionWorld[0] = 0.0;
  this->TextActorPositionWorld[1] = 0.0;
  this->TextActorPositionWorld[2] = 0.0;
  this->TextActorOccluded = false;

  this->AccuratePicker = vtkSmartPointer<vtkCellPicker>::New();
  this->AccuratePicker->SetTolerance(.005);

  // Using the minimum value of -65000 creates a lot of rendering artifacts on the occluded objects, as all of the
  // pixels in the occluded object will have the same depth buffer value (0.0).
  // Using a default value of -25000 strikes a balance between rendering the occluded objects on top of other objects,
  // while still providing enough leeway to ensure that occluded actors are rendered correctly relative to themselves
  // and to other occluded actors.
  this->OccludedRelativeOffset = -25000;
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation3D::~vtkSlicerLinearTransformWidgetRepresentation3D() = default;

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::CanInteract(
  vtkMRMLInteractionEventData* interactionEventData,
  int &foundComponentType, int &foundComponentIndex, double &closestDistance2)
{
  foundComponentType = vtkMRMLTransformDisplayNode::ComponentNone;
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if ( !transformNode || !this->GetVisibility() || !interactionEventData )
    {
    return;
    }

  double displayPosition3[3] = { 0.0, 0.0, 0.0 };
  // Display position is valid in case of desktop interactions. Otherwise it is a 3D only context such as
  // virtual reality, and then we expect a valid world position in the absence of display position.
  if (interactionEventData->IsDisplayPositionValid())
    {
    const int* displayPosition = interactionEventData->GetDisplayPosition();
    displayPosition3[0] = static_cast<double>(displayPosition[0]);
    displayPosition3[1] = static_cast<double>(displayPosition[1]);
    }
  else if (!interactionEventData->IsWorldPositionValid())
    {
    return;
    }

  closestDistance2 = VTK_DOUBLE_MAX; // in display coordinate system (phyisical in case of virtual reality renderer)
  foundComponentIndex = -1;

  // We can interact with the handle if the mouse is hovering over one of the handles (translation or rotation), in display coordinates.
  // If display coordinates for the interaction event are not valid, world coordinates will be checked instead.
  this->CanInteractWithHandles(interactionEventData, foundComponentType, foundComponentIndex, closestDistance2);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::CanInteractWithHandles(
  vtkMRMLInteractionEventData* interactionEventData,
  int& foundComponentType, int& foundComponentIndex, double& closestDistance2)
{
  if (!this->InteractionPipeline || !this->InteractionPipeline->Actor->GetVisibility())
    {
    return;
    }

  double displayPosition3[3] = { 0.0, 0.0, 0.0 };
  // Display position is valid in case of desktop interactions. Otherwise it is a 3D only context such as
  // virtual reality, and then we expect a valid world position in the absence of display position.
  if (interactionEventData->IsDisplayPositionValid())
    {
    const int* displayPosition = interactionEventData->GetDisplayPosition();
    displayPosition3[0] = static_cast<double>(displayPosition[0]);
    displayPosition3[1] = static_cast<double>(displayPosition[1]);
    }
  else if (!interactionEventData->IsWorldPositionValid())
    {
    return;
    }

  bool handlePicked = false;
  vtkSlicerLinearTransformWidgetRepresentation::HandleInfoList handleInfoList = this->InteractionPipeline->GetHandleInfoList();
  for (vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::HandleInfo handleInfo : handleInfoList)
    {
    if (!handleInfo.IsVisible())
      {
      continue;
      }

    double* handleWorldPos = handleInfo.PositionWorld;

    double maxPickingDistanceFromInteractionHandle = this->InteractionPipeline->InteractionHandleSize / 2.0 +
      this->PickingTolerance / interactionEventData->GetWorldToPhysicalScale();
    if (interactionEventData->IsDisplayPositionValid())
      {
      maxPickingDistanceFromInteractionHandle = this->InteractionPipeline->InteractionHandleSize / 2.0
        / this->GetViewScaleFactorAtPosition(handleWorldPos, interactionEventData)
        + this->PickingTolerance * this->ScreenScaleFactor;
      }
    double maxPickingDistanceFromInteractionHandle2 = maxPickingDistanceFromInteractionHandle * maxPickingDistanceFromInteractionHandle;

    double handleDisplayPos[3] = { 0 };

    if (interactionEventData->IsDisplayPositionValid())
      {
      interactionEventData->WorldToDisplay(handleWorldPos, handleDisplayPos);
      double dist2 = vtkMath::Distance2BetweenPoints(handleDisplayPos, displayPosition3);
      if (dist2 < maxPickingDistanceFromInteractionHandle2 && dist2 < closestDistance2)
        {
        closestDistance2 = dist2;
        foundComponentType = handleInfo.ComponentType;
        foundComponentIndex = handleInfo.Index;
        handlePicked = true;
        }
      }
    else
      {
      const double* worldPosition = interactionEventData->GetWorldPosition();
      double dist2 = vtkMath::Distance2BetweenPoints(handleWorldPos, worldPosition);
      if (dist2 < maxPickingDistanceFromInteractionHandle2 && dist2 < closestDistance2)
        {
        closestDistance2 = dist2;
        foundComponentType = handleInfo.ComponentType;
        foundComponentIndex = handleInfo.Index;
        }
      }
    }

  if (!handlePicked)
    {
    // Detect translation handle shaft
    for (vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::HandleInfo handleInfo : handleInfoList)
      {
      if (!handleInfo.IsVisible() || handleInfo.ComponentType != vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
        {
        continue;
        }
      double* handleWorldPos = handleInfo.PositionWorld;
      double handleDisplayPos[3] = { 0 };

      double maxPickingDistanceFromInteractionHandle = this->InteractionPipeline->InteractionHandleSize / 2.0 +
        this->PickingTolerance / interactionEventData->GetWorldToPhysicalScale();
      if (interactionEventData->IsDisplayPositionValid())
        {
        maxPickingDistanceFromInteractionHandle = this->InteractionPipeline->InteractionHandleSize / 2.0
          / this->GetViewScaleFactorAtPosition(handleWorldPos, interactionEventData)
          + this->PickingTolerance * this->ScreenScaleFactor;
        }

      if (interactionEventData->IsDisplayPositionValid())
        {
        interactionEventData->WorldToDisplay(handleWorldPos, handleDisplayPos);

        double originWorldPos[4] = { 0.0, 0.0, 0.0, 1.0 };
        this->InteractionPipeline->GetInteractionHandleOriginWorld(originWorldPos);
        double originDisplayPos[4] = { 0.0 };
        interactionEventData->WorldToDisplay(originWorldPos, originDisplayPos);
        originDisplayPos[2] = displayPosition3[2]; // Handles are always projected
        double t = 0;
        double lineDistance = vtkLine::DistanceToLine(displayPosition3, originDisplayPos, handleDisplayPos, t);
        double lineDistance2 = lineDistance * lineDistance;
        if (lineDistance < maxPickingDistanceFromInteractionHandle && lineDistance2 < closestDistance2)
          {
          closestDistance2 = lineDistance2;
          foundComponentType = handleInfo.ComponentType;
          foundComponentIndex = handleInfo.Index;
          }
        }
      else
        {
        const double* worldPosition = interactionEventData->GetWorldPosition();
        double originWorldPos[4] = { 0.0, 0.0, 0.0, 1.0 };
        this->InteractionPipeline->GetInteractionHandleOriginWorld(originWorldPos);
        double t;
        double lineDistance = vtkLine::DistanceToLine(worldPosition, originWorldPos, handleWorldPos, t);
        if (lineDistance < maxPickingDistanceFromInteractionHandle && lineDistance < closestDistance2)
          {
          closestDistance2 = lineDistance;
          foundComponentType = handleInfo.ComponentType;
          foundComponentIndex = handleInfo.Index;
          }
        }
      }
    }
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::UpdateFromMRMLInternal(vtkMRMLNode* caller, unsigned long event, void *callData /*=nullptr*/)
{
  this->UpdateViewScaleFactor();

  Superclass::UpdateFromMRMLInternal(caller, event, callData);

  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode || !this->IsDisplayable())
    {
    this->VisibilityOff();
    return;
    }

  this->VisibilityOn();

  // Use hierarchy information if any, and if overriding is allowed for the current display node
  double hierarchyOpacity = 1.0;
  if (this->TransformDisplayNode->GetFolderDisplayOverrideAllowed())
    {
    vtkMRMLDisplayableNode* displayableNode = this->TransformDisplayNode->GetDisplayableNode();
    hierarchyOpacity = vtkMRMLFolderDisplayNode::GetHierarchyOpacity(displayableNode);
    }

  /* TODO: implement this for better performance
  if (event == )
    {
    int *nPtr = nullptr;
    int n = -1;
    if (callData != nullptr)
      {
      nPtr = reinterpret_cast<int *>(callData);
      if (nPtr)
        {
        n = *nPtr;
        }
      }
    this->UpdateNthPointAndLabelFromMRML(n);
    }
  else*/
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::GetActors(vtkPropCollection *pc)
{
  Superclass::GetActors(pc);
  this->TextActor->GetActors(pc);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::ReleaseGraphicsResources(
  vtkWindow *win)
{
  Superclass::ReleaseGraphicsResources(win);
  
  this->TextActor->ReleaseGraphicsResources(win);
}

//----------------------------------------------------------------------
int vtkSlicerLinearTransformWidgetRepresentation3D::RenderOverlay(vtkViewport *viewport)
{
  
  int count = Superclass::RenderOverlay(viewport);

  this->TextActorOccluded = false;
  if (this->TextActor->GetVisibility() && this->TransformNode && this->TransformDisplayNode)
    {
    //todo Only show text actor if ,Handle TextActorOccluded

    // Update displayed properties text position from 3D position
    this->Renderer->SetWorldPoint(this->TextActorPositionWorld);
    this->Renderer->WorldToDisplay();
    double textActorPositionDisplay[3] = { 0.0 };
    this->Renderer->GetDisplayPoint(textActorPositionDisplay);
    this->TextActor->SetDisplayPosition(
      static_cast<int>(textActorPositionDisplay[0]),
      static_cast<int>(textActorPositionDisplay[1]));

    if (!this->TextActorOccluded)
      {
      count +=  this->TextActor->RenderOverlay(viewport);
      }
    }
  return count;
}

//-----------------------------------------------------------------------------
int vtkSlicerLinearTransformWidgetRepresentation3D::RenderTranslucentPolygonalGeometry(
  vtkViewport *viewport)
{
  int count = Superclass::RenderTranslucentPolygonalGeometry(viewport);
  
  if (this->TextActor->GetVisibility() && !this->TextActorOccluded)
    {
    count += this->TextActor->RenderTranslucentPolygonalGeometry(viewport);
    }
  return count;
}

//-----------------------------------------------------------------------------
vtkTypeBool vtkSlicerLinearTransformWidgetRepresentation3D::HasTranslucentPolygonalGeometry()
{
  if (this->Superclass::HasTranslucentPolygonalGeometry())
    {
    return true;
    }

  if (this->TextActor->GetVisibility() && !this->TextActorOccluded && this->TextActor->HasTranslucentPolygonalGeometry())
    {
    return true;
    }
  return false;
}

//-----------------------------------------------------------------------------
double *vtkSlicerLinearTransformWidgetRepresentation3D::GetBounds()
{
  this->TransformNode->GetBounds(this->Bounds);
  return this->Bounds;
}

//-----------------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::PrintSelf(ostream& os,
                                                      vtkIndent indent)
{
  //Superclass typedef defined in vtkTypeMacro() found in vtkSetGet.h
  this->Superclass::PrintSelf(os, indent);

  if (this->TextActor)
    {
    os << indent << "Text Visibility: " << this->TextActor->GetVisibility() << "\n";
    }
  else
    {
    os << indent << "Text Visibility: (none)\n";
    }
}

//---------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidgetRepresentation3D::AccuratePick(int x, int y, double pickPoint[3], double pickNormal[3]/*=nullptr*/)
{
  bool success = this->AccuratePicker->Pick(x, y, 0, this->Renderer);
  if (pickNormal)
    {
    this->AccuratePicker->GetPickNormal(pickNormal);
    }
  if (!success)
    {
    return false;
    }

  vtkPoints* pickPositions = this->AccuratePicker->GetPickedPositions();
  vtkIdType numberOfPickedPositions = pickPositions->GetNumberOfPoints();
  if (numberOfPickedPositions<1)
    {
    return false;
    }

  // There may be multiple picked positions, choose the one closest to the camera
  double cameraPosition[3] = { 0,0,0 };
  this->Renderer->GetActiveCamera()->GetPosition(cameraPosition);
  pickPositions->GetPoint(0, pickPoint);
  double minDist2 = vtkMath::Distance2BetweenPoints(pickPoint, cameraPosition);
  for (vtkIdType i = 1; i < numberOfPickedPositions; i++)
    {
    double currentMinDist2 = vtkMath::Distance2BetweenPoints(pickPositions->GetPoint(i), cameraPosition);
    if (currentMinDist2<minDist2)
      {
      pickPositions->GetPoint(i, pickPoint);
      minDist2 = currentMinDist2;
      }
    }
  return true;
}



//----------------------------------------------------------------------
double vtkSlicerLinearTransformWidgetRepresentation3D::GetViewScaleFactorAtPosition(double positionWorld[3], vtkMRMLInteractionEventData* interactionEventData)
{
  double viewScaleFactorMmPerPixel = 1.0;
  if (!this->Renderer || !this->Renderer->GetActiveCamera())
    {
    return viewScaleFactorMmPerPixel;
    }

  vtkCamera * cam = this->Renderer->GetActiveCamera();
  if (cam->GetParallelProjection())
    {
    // Viewport: xmin, ymin, xmax, ymax; range: 0.0-1.0; origin is bottom left
    // Determine the available renderer size in pixels
    double minX = 0;
    double minY = 0;
    this->Renderer->NormalizedDisplayToDisplay(minX, minY);
    double maxX = 1;
    double maxY = 1;
    this->Renderer->NormalizedDisplayToDisplay(maxX, maxY);
    int rendererSizeInPixels[2] = { static_cast<int>(maxX - minX), static_cast<int>(maxY - minY) };
    // Parallel scale: height of the viewport in world-coordinate distances.
    // Larger numbers produce smaller images.
    viewScaleFactorMmPerPixel = (cam->GetParallelScale() * 2.0) / double(rendererSizeInPixels[1]);
    }
  else
    {
    const double cameraFP[] = { positionWorld[0], positionWorld[1], positionWorld[2], 1.0};
    double cameraViewUp[3] = { 0 };
    cam->GetViewUp(cameraViewUp);
    vtkMath::Normalize(cameraViewUp);


    //these should be const but that doesn't compile under VTK 8
    double topCenterWorld[] = {cameraFP[0] + cameraViewUp[0], cameraFP[1] + cameraViewUp[1], cameraFP[2] + cameraViewUp[2], cameraFP[3]};
    double bottomCenterWorld[] = {cameraFP[0] - cameraViewUp[0], cameraFP[1] - cameraViewUp[1], cameraFP[2] - cameraViewUp[2], cameraFP[3]};

    double topCenterDisplay[4];
    double bottomCenterDisplay[4];

    // the WorldToDisplay in interactionEventData is faster if someone has already
    // called it once
    if (interactionEventData)
      {
      interactionEventData->WorldToDisplay(topCenterWorld, topCenterDisplay);
      interactionEventData->WorldToDisplay(bottomCenterWorld, bottomCenterDisplay);
      }
    else
      {
      std::copy(std::begin(topCenterWorld), std::end(topCenterWorld), std::begin(topCenterDisplay));
      this->Renderer->WorldToDisplay(topCenterDisplay[0], topCenterDisplay[1], topCenterDisplay[2]);
      topCenterDisplay[2] = 0.0;

      std::copy(std::begin(bottomCenterWorld), std::end(bottomCenterWorld), std::begin(bottomCenterDisplay));
      this->Renderer->WorldToDisplay(bottomCenterDisplay[0], bottomCenterDisplay[1], bottomCenterDisplay[2]);
      bottomCenterDisplay[2] = 0.0;
      }

    const double distInPixels = sqrt(vtkMath::Distance2BetweenPoints(topCenterDisplay, bottomCenterDisplay));
    // if render window is not initialized yet then distInPixels == 0.0,
    // in that case just leave the default viewScaleFactorMmPerPixel
    if (distInPixels > 1e-3)
      {
      // 2.0 = 2x length of viewUp vector in mm (because viewUp is unit vector)
      viewScaleFactorMmPerPixel = 2.0 / distInPixels;
      }
    }
  return viewScaleFactorMmPerPixel;
}


//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::UpdateViewScaleFactor()
{
  this->ViewScaleFactorMmPerPixel = 1.0;
  this->ScreenSizePixel = 1000.0;
  if (!this->Renderer || !this->Renderer->GetActiveCamera())
    {
    return;
    }

  const int* screenSize = this->Renderer->GetRenderWindow()->GetScreenSize();
  double screenSizePixel = sqrt(screenSize[0] * screenSize[0] + screenSize[1] * screenSize[1]);
  if (screenSizePixel < 1.0)
    {
    // render window is not fully initialized yet
    return;
    }
  this->ScreenSizePixel = screenSizePixel;

  double cameraFP[3] = { 0.0 };
  this->Renderer->GetActiveCamera()->GetFocalPoint(cameraFP);
  this->ViewScaleFactorMmPerPixel = this->GetViewScaleFactorAtPosition(cameraFP);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::UpdateInteractionPipeline()
{
  // Final visibility handled by superclass in vtkSlicerLinearTransformWidgetRepresentation
  Superclass::UpdateInteractionPipeline();
  vtkWarningMacro("Debug!!!!,vtkSlicerLinearTransformWidgetRepresentation3D,UpdateInteractionPipeline");
}

//-----------------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation3D::UpdateRelativeCoincidentTopologyOffsets(vtkMapper* mapper, vtkMapper* occludedMapper)
{
  Superclass::UpdateRelativeCoincidentTopologyOffsets(mapper);

  if (!occludedMapper)
    {
    return;
    }

  Superclass::UpdateRelativeCoincidentTopologyOffsets(occludedMapper);

  /*if (!this->TransformDisplayNode
    || !this->TransformDisplayNode->GetOccludedVisibility()
    || this->TransformDisplayNode->GetOccludedOpacity() <= 0.0)
    {
    return;
    }

  occludedMapper->SetRelativeCoincidentTopologyLineOffsetParameters(-1, this->OccludedRelativeOffset);
  occludedMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(-1, this->OccludedRelativeOffset);
  occludedMapper->SetRelativeCoincidentTopologyPointOffsetParameter(this->OccludedRelativeOffset);*/
}
