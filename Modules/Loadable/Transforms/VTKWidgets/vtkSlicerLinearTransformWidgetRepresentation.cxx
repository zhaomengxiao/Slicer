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
#include "vtkSlicerLinearTransformWidgetRepresentation.h"

#include "vtkCamera.h"
#include "vtkPointData.h"
#include "vtkPointSetToLabelHierarchy.h"
#include "vtkRenderer.h"

// MRML includes

#include <vtkConeSource.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkFocalPlanePointPlacer.h>
#include <vtkLine.h>
#include <vtkMRMLFolderDisplayNode.h>
#include <vtkMRMLTransformNode.h>
#include <vtkRenderWindow.h>

//----------------------------------------------------------------------
//----------------------------------------------------------------------
static const double INTERACTION_HANDLE_SIZE = 3; // Size of the sphere models used for handles
static const double INTERACTION_HANDLE_RADIUS = 0.5; // Size of the sphere models used for handles
static const double INTERACTION_ROTATION_ARC_TUBE_RADIUS = INTERACTION_HANDLE_RADIUS * 0.4; // Radius of the tube connecting the rotation arc.

static const double INTERACTION_WIDGET_RADIUS = INTERACTION_HANDLE_RADIUS * 12.8; // Radius of the entire interaction handle widget.

static const double INTERACTION_ROTATION_ARC_RADIUS = INTERACTION_WIDGET_RADIUS * 0.8; // Radius of the rotation arc.

static const double INTERACTION_TRANSLATION_TIP_RADIUS = INTERACTION_HANDLE_RADIUS * 0.75; // Radius of the arrow tip of the translation handle.
static const double INTERACTION_TRANSLATION_TIP_LENGTH = INTERACTION_TRANSLATION_TIP_RADIUS * 2.0;
static const double INTERACTION_TRANSLATION_HANDLE_SHAFT_RADIUS = INTERACTION_TRANSLATION_TIP_RADIUS * 0.5; // Size of the tube
static const double INTERACTION_TRANSLATION_HANDLE_SHAFT_LENGTH = INTERACTION_HANDLE_RADIUS * 12.8; // Length of the translation handle

vtkStandardNewMacro(vtkSlicerLinearTransformWidgetRepresentation);

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::vtkSlicerLinearTransformWidgetRepresentation()
{
  /*this->ViewScaleFactorMmPerPixel = 1.0;
  this->ScreenSizePixel = 1000;*/
  this->PointPlacer = vtkSmartPointer<vtkFocalPlanePointPlacer>::New();
  this->NeedToRender = false;

  this->AlwaysOnTop = false;

  this->InteractionPipeline = nullptr;

  this->AccuratePicker = vtkSmartPointer<vtkCellPicker>::New();
  this->AccuratePicker->SetTolerance(.005);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::SetupInteractionPipeline()
{
  this->InteractionPipeline = new TransformInteractionPipeline(this);
  InteractionPipeline->InitializePipeline();
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::~vtkSlicerLinearTransformWidgetRepresentation()
{
  if(this->InteractionPipeline != nullptr)
  {
    delete this->InteractionPipeline;
    this->InteractionPipeline = nullptr;
  }

  this->PointPlacer = nullptr;
}

double vtkSlicerLinearTransformWidgetRepresentation::GetViewScaleFactorAtPosition(double positionWorld[3],
  vtkMRMLInteractionEventData* interactionEventData)
{
  double viewScaleFactorMmPerPixel = 1.0;
  if (!this->Renderer || !this->Renderer->GetActiveCamera())
  {
    return viewScaleFactorMmPerPixel;
  }

  vtkCamera* cam = this->Renderer->GetActiveCamera();
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
    const double cameraFP[] = { positionWorld[0], positionWorld[1], positionWorld[2], 1.0 };
    double cameraViewUp[3] = { 0 };
    cam->GetViewUp(cameraViewUp);
    vtkMath::Normalize(cameraViewUp);


    //these should be const but that doesn't compile under VTK 8
    double topCenterWorld[] = { cameraFP[0] + cameraViewUp[0], cameraFP[1] + cameraViewUp[1], cameraFP[2] + cameraViewUp[2], cameraFP[3] };
    double bottomCenterWorld[] = { cameraFP[0] - cameraViewUp[0], cameraFP[1] - cameraViewUp[1], cameraFP[2] - cameraViewUp[2], cameraFP[3] };

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
void vtkSlicerLinearTransformWidgetRepresentation::SetTransformDisplayNode(vtkMRMLTransformDisplayNode *transformDisplayNode)
{
  if (this->TransformDisplayNode == transformDisplayNode)
    {
    return;
    }

  this->TransformDisplayNode = transformDisplayNode;

  vtkMRMLTransformNode* transformNode = nullptr;
  if (this->TransformDisplayNode)
  {
    transformNode = vtkMRMLTransformNode::SafeDownCast(this->TransformDisplayNode->GetDisplayableNode());
  }
  this->SetTransformNode(transformNode);
}

//----------------------------------------------------------------------
vtkMRMLTransformDisplayNode *vtkSlicerLinearTransformWidgetRepresentation::GetTransformDisplayNode()
{
  return this->TransformDisplayNode;
}

//----------------------------------------------------------------------
vtkMRMLTransformNode *vtkSlicerLinearTransformWidgetRepresentation::GetTransformNode()
{
  if (!this->TransformDisplayNode)
    {
    return nullptr;
    }
  return vtkMRMLTransformNode::SafeDownCast(this->TransformDisplayNode->GetDisplayableNode());
}

void vtkSlicerLinearTransformWidgetRepresentation::GetInteractionHandleAxisWorld(int type, int index, double axis[3])
{
  this->InteractionPipeline->GetInteractionHandleAxisWorld(type, index, axis);
}

void vtkSlicerLinearTransformWidgetRepresentation::GetInteractionHandleOriginWorld(double originWorld[3])
{
  {
    if (!originWorld)
    {
      return;
    }

    double handleOrigin[3] = { 0,0,0 };
    this->InteractionPipeline->HandleToWorldTransform->TransformPoint(handleOrigin, originWorld);
  }
}

void vtkSlicerLinearTransformWidgetRepresentation::GetInteractionHandlePositionWorld(int type, int index,
  double position[3])
{
  this->InteractionPipeline->GetInteractionHandlePositionWorld(type, index, position);
}

void vtkSlicerLinearTransformWidgetRepresentation::CanInteract(vtkMRMLInteractionEventData* interactionEventData,
                                                               int& foundComponentType, int& foundComponentIndex, double& closestDistance2)
{
  foundComponentType = vtkMRMLTransformDisplayNode::ComponentNone;
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode || !this->GetVisibility() || !interactionEventData)
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
  if (foundComponentType != vtkMRMLTransformDisplayNode::ComponentNone)
  {
    // if mouse is near a handle then select that (ignore the line + control points)
    return;
  }
  //add other interactor here (line or control points...)
}

void vtkSlicerLinearTransformWidgetRepresentation::CanInteractWithHandles(
  vtkMRMLInteractionEventData* interactionEventData, int& foundComponentType, int& foundComponentIndex,
  double& closestDistance2)
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
    for (TransformInteractionPipeline::HandleInfo handleInfo : handleInfoList)
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

bool vtkSlicerLinearTransformWidgetRepresentation::AccuratePick(int x, int y, double pickPoint[3], double pickNormal[3])
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
  if (numberOfPickedPositions < 1)
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
    if (currentMinDist2 < minDist2)
    {
      pickPositions->GetPoint(i, pickPoint);
      minDist2 = currentMinDist2;
    }
  }
  return true;
}

vtkPointPlacer* vtkSlicerLinearTransformWidgetRepresentation::GetPointPlacer()
{
  return this->PointPlacer;
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::SetTransformNode(vtkMRMLTransformNode *transformNode)
{
  this->TransformNode = transformNode;
}

void vtkSlicerLinearTransformWidgetRepresentation::UpdateViewScaleFactor()
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

  vtkWarningMacro("TransformWidgetRepresentation3D::ScreenSizePixel: " << ScreenSizePixel);
  vtkWarningMacro("TransformWidgetRepresentation3D::ViewScaleFactorMmPerPixel: " << ViewScaleFactorMmPerPixel);
}

void vtkSlicerLinearTransformWidgetRepresentation::UpdateInteractionHandleSize()
{
  if (this->InteractionPipeline)
  {
    //todo handleScale
    this->InteractionPipeline->InteractionHandleSize = this->ScreenSizePixel * this->ScreenScaleFactor
      * INTERACTION_HANDLE_SIZE / 100.0 * this->ViewScaleFactorMmPerPixel;
  }
}

//-----------------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::PrintSelf(ostream& os,
                                                      vtkIndent indent)
{
  //Superclass typedef defined in vtkTypeMacro() found in vtkSetGet.h
  this->Superclass::PrintSelf(os, indent);
  //os << indent << "Point Placer: " << this->PointPlacer << "\n";
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::UpdateFromMRML(
  vtkMRMLNode* caller, unsigned long event, void* callData)
{
  this->UpdateFromMRMLInternal(caller, event, callData);

  if (this->InteractionPipeline == nullptr)
  {
    this->SetupInteractionPipeline();
  }
  if (this->InteractionPipeline != nullptr)
  {
    this->UpdateInteractionPipeline();
  }
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::UpdateFromMRMLInternal(
    vtkMRMLNode* vtkNotUsed(caller), unsigned long event, void *vtkNotUsed(callData))
{
  if (!event || event == vtkMRMLTransformableNode::TransformModifiedEvent)
    {
    this->TransformTransformModifiedTime.Modified();
    }

  if (!event || event == vtkMRMLDisplayableNode::DisplayModifiedEvent)
    {
    // Update MRML data node from display node
    vtkMRMLTransformNode* transformNode = nullptr;
    if (this->TransformDisplayNode)
      {
      transformNode = vtkMRMLTransformNode::SafeDownCast(this->TransformDisplayNode->GetDisplayableNode());
      }
    this->SetTransformNode(transformNode);
    }

  this->NeedToRenderOn(); // TODO: to improve performance, call this only if it is actually needed
  this->UpdateViewScaleFactor();
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::UpdateInteractionPipeline()
{
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode)
    {
    this->InteractionPipeline->Actor->SetVisibility(false);
    return;
    }

  if (!this->TransformDisplayNode)
    {
    this->InteractionPipeline->Actor->SetVisibility(false);
    return;
    }

  Visibility = this->TransformDisplayNode->GetEditorVisibility();
  this->InteractionPipeline->Actor->SetVisibility(Visibility);

  if (Visibility)
  {
    this->InteractionPipeline->UpdateHandleVisibility();
  }

  vtkNew<vtkTransform> handleToWorldTransform;
  vtkNew<vtkMatrix4x4> matrix;
  transformNode->GetMatrixTransformToWorld(matrix);
  handleToWorldTransform->SetMatrix(matrix);
  this->InteractionPipeline->HandleToWorldTransform->DeepCopy(handleToWorldTransform);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::GetActors(vtkPropCollection* pc)
{
  if (this->InteractionPipeline)
  {
    InteractionPipeline->Actor->GetActors(pc);
  }
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::ReleaseGraphicsResources(vtkWindow* window)
{
  if (this->InteractionPipeline)
  {
    InteractionPipeline->Actor->ReleaseGraphicsResources(window);
  }
}

//----------------------------------------------------------------------
int vtkSlicerLinearTransformWidgetRepresentation::RenderOverlay(vtkViewport* viewport)
{
  int count = 0;
  if (this->InteractionPipeline && this->InteractionPipeline->Actor->GetVisibility())
  {
    count += this->InteractionPipeline->Actor->RenderOverlay(viewport);
  }
  return count;
}

//----------------------------------------------------------------------
int vtkSlicerLinearTransformWidgetRepresentation::RenderOpaqueGeometry(vtkViewport* viewport)
{
  this->UpdateViewScaleFactor();
  int count = 0;
  if (this->InteractionPipeline && this->InteractionPipeline->Actor->GetVisibility())
  {
    //update color here not in UpdatePipeline,because here update when render, UpdatePipeline called when MRML modified
    this->InteractionPipeline->UpdateHandleColors();
    if (this->GetTransformDisplayNode())
    {
      this->UpdateInteractionHandleSize();
      this->InteractionPipeline->SetWidgetScale(this->InteractionPipeline->InteractionHandleSize);
    }
    count += this->InteractionPipeline->Actor->RenderOpaqueGeometry(viewport);
  }
  return count;
}

//----------------------------------------------------------------------
int vtkSlicerLinearTransformWidgetRepresentation::RenderTranslucentPolygonalGeometry(vtkViewport* viewport)
{
  int count = 0;
  if (this->InteractionPipeline && this->InteractionPipeline->Actor->GetVisibility())
  {
    this->InteractionPipeline->Actor->SetPropertyKeys(this->GetPropertyKeys());
    count += this->InteractionPipeline->Actor->RenderTranslucentPolygonalGeometry(viewport);
  }
  return count;
}

//----------------------------------------------------------------------
vtkTypeBool vtkSlicerLinearTransformWidgetRepresentation::HasTranslucentPolygonalGeometry()
{
  if (this->InteractionPipeline && this->InteractionPipeline->Actor->GetVisibility() &&
    this->InteractionPipeline->Actor->HasTranslucentPolygonalGeometry())
  {
    return true;
  }
  return false;
}

vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::~TransformInteractionPipeline() = default;

vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::TransformInteractionPipeline(
  vtkMRMLAbstractWidgetRepresentation* representation)
{
  this->Representation = representation;
  /*vtkNew<vtkConeSource> cone;
  cone->SetHeight(7);
  cone->SetRadius(3);
  cone->SetResolution(10);*/
  this->Append = vtkSmartPointer<vtkAppendPolyData>::New();

  this->HandleToWorldTransform = vtkSmartPointer<vtkTransform>::New();
  this->HandleToWorldTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  //this->HandleToWorldTransformFilter->SetInputConnection(cone->GetOutputPort());
  this->HandleToWorldTransformFilter->SetInputConnection(this->Append->GetOutputPort());
  this->HandleToWorldTransformFilter->SetTransform(this->HandleToWorldTransform);

  this->ColorTable = vtkSmartPointer<vtkLookupTable>::New();

  vtkNew<vtkCoordinate> coordinate;
  coordinate->SetCoordinateSystemToWorld();

  this->Mapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
  this->Mapper->SetInputConnection(this->HandleToWorldTransformFilter->GetOutputPort());
  this->Mapper->SetColorModeToMapScalars();
  this->Mapper->ColorByArrayComponent("colorIndex", 0);
  this->Mapper->SetLookupTable(this->ColorTable);
  this->Mapper->ScalarVisibilityOn();
  this->Mapper->UseLookupTableScalarRangeOn();
  this->Mapper->SetTransformCoordinate(coordinate);
  

  //this->Property = vtkSmartPointer<vtkProperty>::New();
  this->Property = vtkSmartPointer<vtkProperty2D>::New();
  this->Property->SetPointSize(0.0);
  this->Property->SetLineWidth(0.0);

  this->Actor = vtkSmartPointer<vtkActor2D>::New();
  //this->Actor = vtkSmartPointer<vtkActor>::New();
  this->Actor->SetProperty(this->Property);
  this->Actor->SetMapper(this->Mapper);
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::InitializePipeline()
{
  this->CreateRotationHandles();
  this->CreateTranslationHandles();
  //this->UpdateHandleVisibility();
  //this->UpdateHandleColors();
  
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::CreateRotationHandles()
{
  this->AxisRotationHandleSource = vtkSmartPointer<vtkSphereSource>::New();
  this->AxisRotationHandleSource->SetRadius(INTERACTION_HANDLE_RADIUS);
  this->AxisRotationHandleSource->SetPhiResolution(16);
  this->AxisRotationHandleSource->SetThetaResolution(16);

  this->AxisRotationArcSource = vtkSmartPointer<vtkArcSource>::New();
  this->AxisRotationArcSource->SetAngle(90);
  this->AxisRotationArcSource->SetCenter(-INTERACTION_ROTATION_ARC_RADIUS, 0, 0);
  this->AxisRotationArcSource->SetPoint1(
    INTERACTION_ROTATION_ARC_RADIUS / sqrt(2) - INTERACTION_ROTATION_ARC_RADIUS,
    -INTERACTION_ROTATION_ARC_RADIUS / sqrt(2), 0);
  this->AxisRotationArcSource->SetPoint2(
    INTERACTION_ROTATION_ARC_RADIUS / sqrt(2) - INTERACTION_ROTATION_ARC_RADIUS,
    INTERACTION_ROTATION_ARC_RADIUS / sqrt(2), 0);
  this->AxisRotationArcSource->SetResolution(16);

  this->AxisRotationTubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
  this->AxisRotationTubeFilter->SetInputConnection(this->AxisRotationArcSource->GetOutputPort());
  this->AxisRotationTubeFilter->SetRadius(INTERACTION_ROTATION_ARC_TUBE_RADIUS);
  this->AxisRotationTubeFilter->SetNumberOfSides(16);
  this->AxisRotationTubeFilter->SetCapping(true);

  vtkNew<vtkPoints> rotationGlyphInteriorAnglePoints;
  rotationGlyphInteriorAnglePoints->InsertNextPoint(this->AxisRotationArcSource->GetPoint1());
  rotationGlyphInteriorAnglePoints->InsertNextPoint(-INTERACTION_ROTATION_ARC_RADIUS, 0, 0);
  rotationGlyphInteriorAnglePoints->InsertNextPoint(this->AxisRotationArcSource->GetPoint2());

  vtkNew<vtkIdList> rotationGlyphInteriorAngleLine;
  rotationGlyphInteriorAngleLine->SetNumberOfIds(3);
  rotationGlyphInteriorAngleLine->SetId(0, 0);
  rotationGlyphInteriorAngleLine->SetId(1, 1);
  rotationGlyphInteriorAngleLine->SetId(2, 2);

  this->AxisRotationInteriorAnglePolyData = vtkSmartPointer<vtkPolyData>::New();
  this->AxisRotationInteriorAnglePolyData->SetPoints(rotationGlyphInteriorAnglePoints);
  this->AxisRotationInteriorAnglePolyData->SetLines(vtkNew<vtkCellArray>());
  this->AxisRotationInteriorAnglePolyData->InsertNextCell(VTK_LINE, rotationGlyphInteriorAngleLine);

  this->AxisRotationInterorAngleTubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
  this->AxisRotationInterorAngleTubeFilter->SetInputData(this->AxisRotationInteriorAnglePolyData);
  this->AxisRotationInterorAngleTubeFilter->SetRadius(INTERACTION_ROTATION_ARC_TUBE_RADIUS);
  this->AxisRotationInterorAngleTubeFilter->SetNumberOfSides(16);

  this->RotationHandlePoints = vtkSmartPointer<vtkPolyData>::New();

  this->RotationScaleTransform = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  this->RotationScaleTransform->SetInputData(this->RotationHandlePoints);
  this->RotationScaleTransform->SetTransform(vtkNew<vtkTransform>());

  this->AxisRotationGlyphSource = vtkSmartPointer <vtkAppendPolyData>::New();
  this->AxisRotationGlyphSource->AddInputConnection(this->AxisRotationHandleSource->GetOutputPort());
  this->AxisRotationGlyphSource->AddInputConnection(this->AxisRotationTubeFilter->GetOutputPort());
  this->AxisRotationGlyphSource->AddInputConnection(this->AxisRotationInterorAngleTubeFilter->GetOutputPort());
  this->AxisRotationGlypher = vtkSmartPointer<vtkTensorGlyph>::New();
  this->AxisRotationGlypher->SetInputConnection(this->RotationScaleTransform->GetOutputPort());
  this->AxisRotationGlypher->SetSourceConnection(this->AxisRotationGlyphSource->GetOutputPort());
  this->AxisRotationGlypher->ScalingOff();
  this->AxisRotationGlypher->ExtractEigenvaluesOff();
  this->AxisRotationGlypher->SetInputArrayToProcess(0, 0, 0, 0, "orientation"); // Orientation direction array

  vtkNew<vtkPoints> points;

  double xRotationHandle[3] = { 0, 1, 1 }; // X-axis
  vtkMath::Normalize(xRotationHandle);
  vtkMath::MultiplyScalar(xRotationHandle, INTERACTION_WIDGET_RADIUS);
  points->InsertNextPoint(xRotationHandle);
  double yRotationHandle[3] = { 1, 0, 1 }; // Y-axis
  vtkMath::Normalize(yRotationHandle);
  vtkMath::MultiplyScalar(yRotationHandle, INTERACTION_WIDGET_RADIUS);
  points->InsertNextPoint(yRotationHandle);
  double zRotationHandle[3] = { 1, 1, 0 }; // Z-axis
  vtkMath::Normalize(zRotationHandle);
  vtkMath::MultiplyScalar(zRotationHandle, INTERACTION_WIDGET_RADIUS);
  points->InsertNextPoint(zRotationHandle);
  this->RotationHandlePoints->SetPoints(points);

  vtkNew<vtkDoubleArray> orientationArray;
  orientationArray->SetName("orientation");
  orientationArray->SetNumberOfComponents(9);
  vtkNew<vtkTransform> xRotationOrientation;
  xRotationOrientation->RotateX(90);
  xRotationOrientation->RotateY(90);
  xRotationOrientation->RotateZ(45);
  vtkMatrix4x4* xRotationMatrix = xRotationOrientation->GetMatrix();
  orientationArray->InsertNextTuple9(xRotationMatrix->GetElement(0, 0), xRotationMatrix->GetElement(1, 0), xRotationMatrix->GetElement(2, 0),
    xRotationMatrix->GetElement(0, 1), xRotationMatrix->GetElement(1, 1), xRotationMatrix->GetElement(2, 1),
    xRotationMatrix->GetElement(0, 2), xRotationMatrix->GetElement(1, 2), xRotationMatrix->GetElement(2, 2));
  vtkNew<vtkTransform> yRotationOrientation;
  yRotationOrientation->RotateX(90);
  yRotationOrientation->RotateZ(45);
  vtkMatrix4x4* yRotationMatrix = yRotationOrientation->GetMatrix();
  orientationArray->InsertNextTuple9(yRotationMatrix->GetElement(0, 0), yRotationMatrix->GetElement(1, 0), yRotationMatrix->GetElement(2, 0),
    yRotationMatrix->GetElement(0, 1), yRotationMatrix->GetElement(1, 1), yRotationMatrix->GetElement(2, 1),
    yRotationMatrix->GetElement(0, 2), yRotationMatrix->GetElement(1, 2), yRotationMatrix->GetElement(2, 2));
  vtkNew<vtkTransform> zRotationOrientation;
  zRotationOrientation->RotateZ(45);
  vtkMatrix4x4* zRotationMatrix = zRotationOrientation->GetMatrix();
  orientationArray->InsertNextTuple9(zRotationMatrix->GetElement(0, 0), zRotationMatrix->GetElement(1, 0), zRotationMatrix->GetElement(2, 0),
    zRotationMatrix->GetElement(0, 1), zRotationMatrix->GetElement(1, 1), zRotationMatrix->GetElement(2, 1),
    zRotationMatrix->GetElement(0, 2), zRotationMatrix->GetElement(1, 2), zRotationMatrix->GetElement(2, 2));
  this->RotationHandlePoints->GetPointData()->AddArray(orientationArray);

  vtkNew<vtkIdTypeArray> visibilityArray;
  visibilityArray->SetName("visibility");
  visibilityArray->SetNumberOfComponents(1);
  visibilityArray->SetNumberOfValues(this->RotationHandlePoints->GetNumberOfPoints());
  visibilityArray->Fill(1);
  this->RotationHandlePoints->GetPointData()->AddArray(visibilityArray);

  this->Append->AddInputConnection(this->AxisRotationGlypher->GetOutputPort());
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::CreateTranslationHandles()
{
  this->AxisTranslationGlyphSource = vtkSmartPointer<vtkArrowSource>::New();
  this->AxisTranslationGlyphSource->SetTipRadius(INTERACTION_TRANSLATION_TIP_RADIUS);
  this->AxisTranslationGlyphSource->SetTipLength(INTERACTION_TRANSLATION_TIP_LENGTH / INTERACTION_WIDGET_RADIUS); // Scaled by INTERACTION_WIDGET_RADIUS later
  this->AxisTranslationGlyphSource->SetShaftRadius(INTERACTION_TRANSLATION_HANDLE_SHAFT_RADIUS);
  this->AxisTranslationGlyphSource->SetTipResolution(16);
  this->AxisTranslationGlyphSource->SetShaftResolution(16);
  this->AxisTranslationGlyphSource->InvertOn();

  vtkNew<vtkTransform> translationArrowGlyphTransform;
  translationArrowGlyphTransform->Translate(INTERACTION_HANDLE_RADIUS, 0, 0); // Move away from the origin so that it doesn't overlap with the center handle
  translationArrowGlyphTransform->Scale(INTERACTION_WIDGET_RADIUS, 1.0, 1.0); // Increase arrow length to INTERACTION_WIDGET_RADIUS
  translationArrowGlyphTransform->RotateY(180); // Flip so that the arrow is facing in the +ve X direction.

  this->AxisTranslationGlyphTransformer = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  this->AxisTranslationGlyphTransformer->SetTransform(translationArrowGlyphTransform);
  this->AxisTranslationGlyphTransformer->SetInputConnection(this->AxisTranslationGlyphSource->GetOutputPort());

  this->TranslationHandlePoints = vtkSmartPointer<vtkPolyData>::New();

  this->TranslationScaleTransform = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  this->TranslationScaleTransform->SetInputData(this->TranslationHandlePoints);
  this->TranslationScaleTransform->SetTransform(vtkNew<vtkTransform>());

  this->AxisTranslationGlypher = vtkSmartPointer<vtkGlyph3D>::New();
  this->AxisTranslationGlypher->SetInputConnection(this->TranslationScaleTransform->GetOutputPort());
  this->AxisTranslationGlypher->SetSourceConnection(0, this->AxisTranslationGlyphTransformer->GetOutputPort());
  this->AxisTranslationGlypher->SetSourceConnection(1, this->AxisRotationHandleSource->GetOutputPort());
  this->AxisTranslationGlypher->ScalingOn();
  this->AxisTranslationGlypher->SetScaleModeToDataScalingOff();
  this->AxisTranslationGlypher->SetIndexModeToScalar();
  this->AxisTranslationGlypher->SetColorModeToColorByScalar();
  this->AxisTranslationGlypher->OrientOn();
  this->AxisTranslationGlypher->SetInputArrayToProcess(0, 0, 0, 0, "glyphIndex"); // Glyph shape
  this->AxisTranslationGlypher->SetInputArrayToProcess(1, 0, 0, 0, "orientation"); // Orientation direction array

  vtkNew<vtkPoints> points;
  points->InsertNextPoint(INTERACTION_WIDGET_RADIUS, 0, 0); // X-axis
  points->InsertNextPoint(0, INTERACTION_WIDGET_RADIUS, 0); // Y-axis
  points->InsertNextPoint(0, 0, INTERACTION_WIDGET_RADIUS); // Z-axis
  points->InsertNextPoint(0, 0, 0); // View plane translation
  this->TranslationHandlePoints->SetPoints(points);

  vtkNew<vtkDoubleArray> orientationArray;
  orientationArray->SetName("orientation");
  orientationArray->SetNumberOfComponents(3);
  orientationArray->InsertNextTuple3(1, 0, 0);
  orientationArray->InsertNextTuple3(0, 1, 0);
  orientationArray->InsertNextTuple3(0, 0, 1);
  orientationArray->InsertNextTuple3(1, 0, 0); // View plane translation
  this->TranslationHandlePoints->GetPointData()->AddArray(orientationArray);

  vtkNew<vtkDoubleArray> glyphIndexArray;
  glyphIndexArray->SetName("glyphIndex");
  glyphIndexArray->SetNumberOfComponents(1);
  glyphIndexArray->InsertNextTuple1(0);
  glyphIndexArray->InsertNextTuple1(0);
  glyphIndexArray->InsertNextTuple1(0);
  glyphIndexArray->InsertNextTuple1(1);
  this->TranslationHandlePoints->GetPointData()->AddArray(glyphIndexArray);

  vtkNew<vtkIdTypeArray> visibilityArray;
  visibilityArray->SetName("visibility");
  visibilityArray->SetNumberOfComponents(1);
  visibilityArray->SetNumberOfValues(this->TranslationHandlePoints->GetNumberOfPoints());
  visibilityArray->Fill(1);
  this->TranslationHandlePoints->GetPointData()->AddArray(visibilityArray);

  this->Append->AddInputConnection(this->AxisTranslationGlypher->GetOutputPort());
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::UpdateHandleVisibility()
{
  vtkSlicerLinearTransformWidgetRepresentation* rep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->Representation);
  vtkMRMLTransformDisplayNode* displayNode = nullptr;
  if (rep)
  {
    displayNode = rep->GetTransformDisplayNode();
  }
  if (!displayNode)
  {
    vtkGenericWarningMacro("UpdateHandleVisibility: Invalid display node");
    return;
  }

  vtkIdTypeArray* rotationVisibilityArray = vtkIdTypeArray::SafeDownCast(this->RotationHandlePoints->GetPointData()->GetArray("visibility"));
  if (rotationVisibilityArray)
  {
    bool rotationVisibility = displayNode->GetEditorRotationEnabled();
    rotationVisibilityArray->SetValue(0, rotationVisibility);
    rotationVisibilityArray->SetValue(1, rotationVisibility);
    rotationVisibilityArray->SetValue(2, rotationVisibility);
  }

  vtkIdTypeArray* translationVisibilityArray = vtkIdTypeArray::SafeDownCast(this->TranslationHandlePoints->GetPointData()->GetArray("visibility"));
  if (translationVisibilityArray)
  {
    bool translationVisibility = displayNode->GetEditorTranslationEnabled();
    translationVisibilityArray->SetValue(0, translationVisibility);
    translationVisibilityArray->SetValue(1, translationVisibility);
    translationVisibilityArray->SetValue(2, translationVisibility);
    translationVisibilityArray->SetValue(3, translationVisibility);
  }
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::UpdateHandleColors()
{
  if (!this->ColorTable)
  {
    return;
  }

  int numberOfHandles = this->RotationHandlePoints->GetNumberOfPoints()
    + this->TranslationHandlePoints->GetNumberOfPoints();
    /*+ this->ScaleHandlePoints->GetNumberOfPoints();*/
  this->ColorTable->SetNumberOfTableValues(numberOfHandles);
  this->ColorTable->SetTableRange(0, numberOfHandles - 1);

  int colorIndex = 0;
  double color[4] = { 0.0, 0.0, 0.0, 0.0 };

  // Rotation handles
  vtkSmartPointer<vtkFloatArray> rotationColorArray = vtkFloatArray::SafeDownCast(
    this->RotationHandlePoints->GetPointData()->GetAbstractArray("colorIndex"));
  if (!rotationColorArray)
  {
    rotationColorArray = vtkSmartPointer<vtkFloatArray>::New();
    rotationColorArray->SetName("colorIndex");
    rotationColorArray->SetNumberOfComponents(1);
    this->RotationHandlePoints->GetPointData()->AddArray(rotationColorArray);
    this->RotationHandlePoints->GetPointData()->SetActiveScalars("colorIndex");
  }
  rotationColorArray->Initialize();
  rotationColorArray->SetNumberOfTuples(this->RotationHandlePoints->GetNumberOfPoints());
  for (int i = 0; i < this->RotationHandlePoints->GetNumberOfPoints(); ++i)
  {
    this->GetHandleColor(vtkMRMLTransformDisplayNode::ComponentRotationHandle, i, color);
    this->ColorTable->SetTableValue(colorIndex, color);
    rotationColorArray->SetTuple1(i, colorIndex);
    ++colorIndex;
  }

  // Translation handles
  vtkSmartPointer<vtkFloatArray> translationColorArray = vtkFloatArray::SafeDownCast(
    this->TranslationHandlePoints->GetPointData()->GetAbstractArray("colorIndex"));
  if (!translationColorArray)
  {
    translationColorArray = vtkSmartPointer<vtkFloatArray>::New();
    translationColorArray->SetName("colorIndex");
    translationColorArray->SetNumberOfComponents(1);
    this->TranslationHandlePoints->GetPointData()->AddArray(translationColorArray);
    this->TranslationHandlePoints->GetPointData()->SetActiveScalars("colorIndex");
  }
  translationColorArray->Initialize();
  translationColorArray->SetNumberOfTuples(this->TranslationHandlePoints->GetNumberOfPoints());
  for (int i = 0; i < this->TranslationHandlePoints->GetNumberOfPoints(); ++i)
  {
    this->GetHandleColor(vtkMRMLTransformDisplayNode::ComponentTranslationHandle, i, color);
    this->ColorTable->SetTableValue(colorIndex, color);
    translationColorArray->SetTuple1(i, colorIndex);
    ++colorIndex;
  }

  // Scale handles
  /*vtkSmartPointer<vtkFloatArray> scaleColorArray = vtkFloatArray::SafeDownCast(
    this->ScaleHandlePoints->GetPointData()->GetAbstractArray("colorIndex"));
  if (!scaleColorArray)
  {
    scaleColorArray = vtkSmartPointer<vtkFloatArray>::New();
    scaleColorArray->SetName("colorIndex");
    scaleColorArray->SetNumberOfComponents(1);
    this->ScaleHandlePoints->GetPointData()->AddArray(scaleColorArray);
    this->ScaleHandlePoints->GetPointData()->SetActiveScalars("colorIndex");
  }
  scaleColorArray->Initialize();
  scaleColorArray->SetNumberOfTuples(this->ScaleHandlePoints->GetNumberOfPoints());
  for (int i = 0; i < this->ScaleHandlePoints->GetNumberOfPoints(); ++i)
  {
    this->GetHandleColor(vtkMRMLtransformDisplayNode::ComponentScaleHandle, i, color);
    this->ColorTable->SetTableValue(colorIndex, color);
    scaleColorArray->SetTuple1(i, colorIndex);
    ++colorIndex;
  }*/

  this->ColorTable->Build();
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetHandleColor(int type, int index,
  double color[4])
{
  if (!color)
  {
    return;
  }

  double red[4] = { 1.00, 0.00, 0.00, 1.00 };
  double green[4] = { 0.00, 1.00, 0.00, 1.00 };
  double blue[4] = { 0.00, 0.00, 1.00, 1.00 };
  double orange[4] = { 1.00, 0.50, 0.00, 1.00 };
  double white[4] = { 1.00, 1.00, 1.00, 1.00 };
  double yellow[4] = { 1.00, 1.00, 0.00, 1.00 };

  double* currentColor = red;
  switch (index)
  {
  case 0:
    currentColor = red;
    break;
  case 1:
    currentColor = green;
    break;
  case 2:
    currentColor = blue;
    break;
  case 3:
    currentColor = orange;
    break;
  default:
    currentColor = white;
    break;
  }

  vtkSlicerLinearTransformWidgetRepresentation* rep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->Representation);
  vtkMRMLTransformDisplayNode* displayNode = nullptr;
  if (rep)
  {
    displayNode = rep->GetTransformDisplayNode();
  }

  double opacity = this->GetHandleOpacity(type, index);
  if (displayNode && displayNode->GetActiveComponentType() == type && displayNode->GetActiveComponentIndex() == index)
  {
    currentColor = yellow;
    opacity = 1.0;
  }

  for (int i = 0; i < 3; ++i)
  {
    color[i] = currentColor[i];
  }

  vtkPolyData* handlePoints = nullptr;
  if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
  {
    handlePoints = this->TranslationHandlePoints;
  }
  else if (type == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
  {
    handlePoints = this->RotationHandlePoints;
  }
  /*else if (type == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
  {
    handlePoints = this->ScaleHandlePoints;
  }*/

  vtkIdTypeArray* visibilityArray = nullptr;
  if (handlePoints)
  {
    visibilityArray = vtkIdTypeArray::SafeDownCast(handlePoints->GetPointData()->GetArray("visibility"));
  }

  if (visibilityArray)
  {
    opacity = visibilityArray->GetValue(index) ? opacity : 0.0;
  }
  color[3] = opacity;
}

double vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetHandleOpacity(int type, int index)
{
  // Determine if the handle should be displayed
  bool handleVisible = true;
  vtkSlicerLinearTransformWidgetRepresentation* rep = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->Representation);
  vtkMRMLTransformDisplayNode* displayNode = nullptr;
  if (rep)
  {
    displayNode = rep->GetTransformDisplayNode();
  }
  if (displayNode)
  {
    handleVisible = displayNode->GetHandleVisibility(type);
  }
  if (!handleVisible)
  {
    return 0.0;
  }

  double opacity = 1.0;
  if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle && index == 3)
  {
    // View plane transform handle is always visible regardless of angle
    return opacity;
  }

  double viewNormal[3] = { 0.0, 0.0, 0.0 };
  this->GetViewPlaneNormal(viewNormal);

  double axis[3] = { 0.0, 0.0, 0.0 };
  this->GetInteractionHandleAxisWorld(type, index, axis);
  if (vtkMath::Dot(viewNormal, axis) < 0)
  {
    vtkMath::MultiplyScalar(axis, -1);
  }

  double fadeAngleRange = this->StartFadeAngle - this->EndFadeAngle;
  double angle = vtkMath::DegreesFromRadians(vtkMath::AngleBetweenVectors(viewNormal, axis));
  if (type == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
  {
    // Fade happens when the axis approaches 90 degrees from the view normal
    if (angle > 90 - this->EndFadeAngle)
    {
      opacity = 0.0;
    }
    else if (angle > 90 - this->StartFadeAngle)
    {
      double difference = angle - (90 - this->StartFadeAngle);
      opacity = 1.0 - (difference / fadeAngleRange);
    }
  }
  else if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle || type == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
  {
    // Fade happens when the axis approaches 0 degrees from the view normal
    if (angle < this->EndFadeAngle)
    {
      opacity = 0.0;
    }
    else if (angle < this->StartFadeAngle)
    {
      double difference = angle - this->EndFadeAngle;
      opacity = (difference / fadeAngleRange);
    }
  }
  return opacity;
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetViewPlaneNormal(double normal[3])
{
  if (!normal)
  {
    return;
  }
  if (this->Representation && this->Representation->GetRenderer() && this->Representation->GetRenderer()->GetActiveCamera())
  {
    vtkCamera* camera = this->Representation->GetRenderer()->GetActiveCamera();
    camera->GetViewPlaneNormal(normal);
  }
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandleAxisWorld(int type,
  int index, double axisWorld[3])
{
  if (!axisWorld)
  {
    vtkErrorWithObjectMacro(nullptr, "GetInteractionHandleVectorWorld: Invalid axis argument!");
    return;
  }

  axisWorld[0] = 0.0;
  axisWorld[1] = 0.0;
  axisWorld[2] = 0.0;

  if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
  {
    switch (index)
    {
    case 0:
      axisWorld[0] = 1.0;
      break;
    case 1:
      axisWorld[1] = 1.0;
      break;
    case 2:
      axisWorld[2] = 1.0;
      break;
    default:
      break;
    }
  }
  else if (type == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
  {
    switch (index)
    {
    case 0:
      axisWorld[0] = 1.0;
      break;
    case 1:
      axisWorld[1] = 1.0;
      break;
    case 2:
      axisWorld[2] = 1.0;
      break;
    default:
      break;
    }
  }
  else if (type == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
  {
    switch (index)
    {
    case 0:
      axisWorld[0] = 1.0;
      break;
    case 1:
      axisWorld[1] = 1.0;
      break;
    case 2:
      axisWorld[2] = 1.0;
      break;
    default:
      break;
    }
  }
  double origin[3] = { 0.0, 0.0, 0.0 };
  this->HandleToWorldTransform->TransformVectorAtPoint(origin, axisWorld, axisWorld);
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandleOriginWorld(
  double originWorld[3])
{
  if (!originWorld)
  {
    return;
  }

  double handleOrigin[3] = { 0,0,0 };
  this->HandleToWorldTransform->TransformPoint(handleOrigin, originWorld);
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandlePositionWorld(
  int type, int index, double positionWorld[3])
{
  if (!positionWorld)
  {
    vtkErrorWithObjectMacro(nullptr, "GetInteractionHandlePositionWorld: Invalid position argument!");
  }

  if (type == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
  {
    this->RotationHandlePoints->GetPoint(index, positionWorld);
    this->RotationScaleTransform->GetTransform()->TransformPoint(positionWorld, positionWorld);
    this->HandleToWorldTransform->TransformPoint(positionWorld, positionWorld);
  }
  else if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
  {
    this->TranslationHandlePoints->GetPoint(index, positionWorld);
    this->TranslationScaleTransform->GetTransform()->TransformPoint(positionWorld, positionWorld);
    this->HandleToWorldTransform->TransformPoint(positionWorld, positionWorld);
  }
  else if (type == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
  {
    /*this->ScaleHandlePoints->GetPoint(index, positionWorld);
    this->HandleToWorldTransform->TransformPoint(positionWorld, positionWorld);*/
    vtkErrorWithObjectMacro(nullptr,"TransformInteractionPipeline::GetInteractionHandlePositionWorld::ComponentScaleHandle not implant yet");
  }
}

void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::SetWidgetScale(double scale)
{
  vtkNew<vtkTransform> scaleTransform;
  scaleTransform->Scale(scale, scale, scale);
  this->RotationScaleTransform->SetTransform(scaleTransform);
  this->TranslationScaleTransform->SetTransform(scaleTransform);
  //this->ScaleScaleTransform->SetTransform(scaleTransform);
  this->AxisRotationGlypher->SetScaleFactor(scale);
  this->AxisTranslationGlypher->SetScaleFactor(scale);
  //this->AxisScaleGlypher->SetScaleFactor(scale);
}

vtkSlicerLinearTransformWidgetRepresentation::HandleInfoList vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetHandleInfoList()
{
  HandleInfoList handleInfoList;
  for (int i = 0; i < this->RotationHandlePoints->GetNumberOfPoints(); ++i)
  {
    double handlePositionLocal[3] = { 0 };
    double handlePositionWorld[3] = { 0 };
    this->RotationHandlePoints->GetPoint(i, handlePositionLocal);
    this->RotationScaleTransform->GetTransform()->TransformPoint(handlePositionLocal, handlePositionWorld);
    this->HandleToWorldTransform->TransformPoint(handlePositionWorld, handlePositionWorld);
    double color[4] = { 0 };
    this->GetHandleColor(vtkMRMLTransformDisplayNode::ComponentRotationHandle, i, color);
    HandleInfo info(i, vtkMRMLTransformDisplayNode::ComponentRotationHandle, handlePositionWorld, handlePositionLocal, color);
    handleInfoList.push_back(info);
  }

  for (int i = 0; i < this->TranslationHandlePoints->GetNumberOfPoints(); ++i)
  {
    double handlePositionLocal[3] = { 0 };
    double handlePositionWorld[3] = { 0 };
    this->TranslationHandlePoints->GetPoint(i, handlePositionLocal);
    this->TranslationScaleTransform->GetTransform()->TransformPoint(handlePositionLocal, handlePositionWorld);
    this->HandleToWorldTransform->TransformPoint(handlePositionWorld, handlePositionWorld);
    double color[4] = { 0 };
    this->GetHandleColor(vtkMRMLTransformDisplayNode::ComponentTranslationHandle, i, color);
    HandleInfo info(i, vtkMRMLTransformDisplayNode::ComponentTranslationHandle, handlePositionWorld, handlePositionLocal, color);
    handleInfoList.push_back(info);
  }

  /*for (int i = 0; i < this->ScaleHandlePoints->GetNumberOfPoints(); ++i)
  {
    double handlePositionLocal[3] = { 0 };
    double handlePositionWorld[3] = { 0 };
    this->ScaleHandlePoints->GetPoint(i, handlePositionLocal);
    this->ScaleScaleTransform->GetTransform()->TransformPoint(handlePositionLocal, handlePositionWorld);
    this->HandleToWorldTransform->TransformPoint(handlePositionWorld, handlePositionWorld);
    double color[4] = { 0 };
    this->GetHandleColor(vtkMRMLTransformDisplayNode::ComponentScaleHandle, i, color);
    HandleInfo info(i, vtkMRMLTransformDisplayNode::ComponentScaleHandle, handlePositionWorld, handlePositionLocal, color);
    handleInfoList.push_back(info);
  }*/

  return handleInfoList;
}
