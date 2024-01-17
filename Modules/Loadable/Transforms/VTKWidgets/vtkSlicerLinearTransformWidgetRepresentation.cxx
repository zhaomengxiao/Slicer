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
#include "vtkAppendPolyData.h"
#include "vtkArcSource.h"
#include "vtkArrowSource.h"
#include "vtkSlicerLinearTransformWidgetRepresentation.h"
#include "vtkCamera.h"
#include "vtkDoubleArray.h"
#include "vtkFloatArray.h"
#include "vtkFocalPlanePointPlacer.h"
#include "vtkGlyph3D.h"
#include "vtkLine.h"
#include "vtkLineSource.h"
#include "vtkLookupTable.h"
#include "vtkMRMLSliceNode.h"
#include "vtkMRMLViewNode.h"
#include "vtkPointData.h"
#include "vtkPointSetToLabelHierarchy.h"
#include "vtkPolyDataMapper2D.h"
#include "vtkProperty2D.h"
#include "vtkRenderer.h"
#include "vtkSphereSource.h"
#include "vtkStringArray.h"
#include "vtkTextActor.h"
#include "vtkTextProperty.h"
#include "vtkTensorGlyph.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTubeFilter.h"

// MRML includes
#include <vtkMRMLFolderDisplayNode.h>
#include <vtkMRMLInteractionEventData.h>
#include <vtkMRMLTransformNode.h>

//----------------------------------------------------------------------
static const double INTERACTION_HANDLE_RADIUS = 0.5; // Size of the sphere models used for handles
static const double INTERACTION_ROTATION_ARC_TUBE_RADIUS = INTERACTION_HANDLE_RADIUS * 0.4; // Radius of the tube connecting the rotation arc.

static const double INTERACTION_WIDGET_RADIUS = INTERACTION_HANDLE_RADIUS * 12.8; // Radius of the entire interaction handle widget.

static const double INTERACTION_ROTATION_ARC_RADIUS = INTERACTION_WIDGET_RADIUS * 0.8; // Radius of the rotation arc.

static const double INTERACTION_TRANSLATION_TIP_RADIUS = INTERACTION_HANDLE_RADIUS * 0.75; // Radius of the arrow tip of the translation handle.
static const double INTERACTION_TRANSLATION_TIP_LENGTH = INTERACTION_TRANSLATION_TIP_RADIUS * 2.0;
static const double INTERACTION_TRANSLATION_HANDLE_SHAFT_RADIUS = INTERACTION_TRANSLATION_TIP_RADIUS * 0.5; // Size of the tube
static const double INTERACTION_TRANSLATION_HANDLE_SHAFT_LENGTH = INTERACTION_HANDLE_RADIUS * 12.8; // Length of the translation handle



//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::vtkSlicerLinearTransformWidgetRepresentation()
{
  this->ViewScaleFactorMmPerPixel = 1.0;
  this->ScreenSizePixel = 1000;

  this->NeedToRender = false;
  this->CurveClosed = 0;

  this->TextActor = vtkSmartPointer<vtkTextActor>::New();
  // hide by default, if a concrete class implements properties display, it will enable it
  this->TextActor->SetVisibility(false);

  this->PointPlacer = vtkSmartPointer<vtkFocalPlanePointPlacer>::New();

  this->AlwaysOnTop = false;

  this->InteractionPipeline = nullptr;
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::SetupInteractionPipeline()
{
  this->InteractionPipeline = new TransformInteractionPipeline(this);
  this->InteractionPipeline->InitializePipeline();
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::~vtkSlicerLinearTransformWidgetRepresentation()
{
  // Force deleting variables to prevent circular dependency keeping objects alive
  this->PointPlacer = nullptr;

  if (this->InteractionPipeline != nullptr)
    {
    delete this->InteractionPipeline;
    this->InteractionPipeline = nullptr;
    }
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

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::SetTransformNode(vtkMRMLTransformNode *transformNode)
{
  this->TransformNode = transformNode;
}

//-----------------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::PrintSelf(ostream& os,
                                                      vtkIndent indent)
{
  //Superclass typedef defined in vtkTypeMacro() found in vtkSetGet.h
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Point Placer: " << this->PointPlacer << "\n";
}

//-----------------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::CanInteract(
  vtkMRMLInteractionEventData* vtkNotUsed(interactionEventData),
  int &foundComponentType, int &vtkNotUsed(foundComponentIndex), double &vtkNotUsed(closestDistance2))
{
  foundComponentType = vtkMRMLTransformDisplayNode::ComponentNone;
}

vtkPointPlacer* vtkSlicerLinearTransformWidgetRepresentation::GetPointPlacer()
{
  return this->PointPlacer;
}

//----------------------------------------------------------------------
bool vtkSlicerLinearTransformWidgetRepresentation::GetTransformationReferencePoint(double referencePointWorld[3])
{
  vtkMRMLTransformNode* transformNode = this->GetTransformNode();
  if (!transformNode)
    {
    return false;
    }

  vtkSmartPointer<vtkMatrix4x4> matrix = vtkMatrix4x4::New();
  transformNode->GetMatrixTransformToWorld(matrix);
  
  referencePointWorld[0] = matrix->GetElement(0, 3);
  referencePointWorld[1] = matrix->GetElement(1, 3);
  referencePointWorld[2] = matrix->GetElement(2, 3);

  return true;
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::UpdateFromMRML(
  vtkMRMLNode* caller, unsigned long event, void* callData)
{
  this->UpdateFromMRMLInternal(caller, event, callData);

  if (!this->InteractionPipeline)
    {
    this->SetupInteractionPipeline();
    }
  if (this->InteractionPipeline)
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

  if (this->TransformNode)
    {
    std::string labelText;
    if (this->TransformNode->GetName())
      {
      labelText = this->TransformNode->GetName();
      }

    this->TextActor->SetInput(labelText.c_str());
    }
  else
    {
    this->TextActor->SetInput("");
    }


  this->NeedToRenderOn(); // TODO: to improve performance, call this only if it is actually needed
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

  this->InteractionPipeline->Actor->SetVisibility(this->TransformDisplayNode->GetEditorVisibility());
  this->InteractionPipeline->UpdateHandleVisibility();

  vtkNew<vtkTransform> handleToWorldTransform;
  vtkNew<vtkMatrix4x4> matrix;
  transformNode->GetMatrixTransformToWorld(matrix);
  handleToWorldTransform->SetMatrix(matrix);
  this->InteractionPipeline->HandleToWorldTransform->DeepCopy(handleToWorldTransform);
}


//-----------------------------------------------------------------------------
bool vtkSlicerLinearTransformWidgetRepresentation::IsDisplayable()
{
  if (!this->TransformDisplayNode
    || !this->ViewNode
    || !this->TransformDisplayNode->GetVisibility()
    || !this->TransformDisplayNode->IsDisplayableInView(this->ViewNode->GetID()))
    {
    return false;
    }

  // If parent folder visibility is set to false then the Transform is not visible
  if (this->TransformDisplayNode->GetFolderDisplayOverrideAllowed())
    {
    vtkMRMLDisplayableNode* displayableNode = this->TransformDisplayNode->GetDisplayableNode();
    // Visibility is applied regardless the fact whether there is override or not.
    // Visibility of items defined by hierarchy is off if any of the ancestors is explicitly hidden.
    // However, this does not apply on display nodes that do not allow overrides (FolderDisplayOverrideAllowed)
    if (!vtkMRMLFolderDisplayNode::GetHierarchyVisibility(displayableNode))
      {
      return false;
      }
    }
  if (vtkMRMLSliceNode::SafeDownCast(this->ViewNode))
    {
    if (!this->TransformDisplayNode->GetVisibility2D())
      {
      return false;
      }
    }
  if (vtkMRMLViewNode::SafeDownCast(this->ViewNode))
    {
    if (!this->TransformDisplayNode->GetVisibility3D())
      {
      return false;
      }
    }
  return true;
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::GetActors(vtkPropCollection* pc)
{
  if (this->InteractionPipeline)
    {
    this->InteractionPipeline->Actor->GetActors(pc);
    }
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::ReleaseGraphicsResources(vtkWindow* window)
{
  if (this->InteractionPipeline)
    {
    this->InteractionPipeline->Actor->ReleaseGraphicsResources(window);
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
  int count = 0;
  if (this->InteractionPipeline && this->InteractionPipeline->Actor->GetVisibility())
    {
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

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::GetInteractionHandleAxisWorld(int type, int index, double axis[3])
{
  this->InteractionPipeline->GetInteractionHandleAxisWorld(type, index, axis);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::GetInteractionHandleOriginWorld(double origin[3])
{
  this->InteractionPipeline->GetInteractionHandleOriginWorld(origin);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::GetInteractionHandlePositionWorld(int type, int index, double position[3])
{
  this->InteractionPipeline->GetInteractionHandlePositionWorld(type, index, position);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::UpdateInteractionHandleSize()
{
  if (this->InteractionPipeline)
    {
    this->InteractionPipeline->InteractionHandleSize = this->ScreenSizePixel * this->ScreenScaleFactor
      * this->TransformDisplayNode->GetGlyphScalePercent() / 100.0 * this->ViewScaleFactorMmPerPixel;
    }
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::TransformInteractionPipeline(vtkMRMLAbstractWidgetRepresentation* representation)
{
  this->Representation = representation;

  this->Append = vtkSmartPointer<vtkAppendPolyData>::New();

  this->HandleToWorldTransform = vtkSmartPointer<vtkTransform>::New();
  this->HandleToWorldTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
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

  this->Property = vtkSmartPointer<vtkProperty2D>::New();
  this->Property->SetPointSize(0.0);
  this->Property->SetLineWidth(0.0);

  this->Actor = vtkSmartPointer<vtkActor2D>::New();
  this->Actor->SetProperty(this->Property);
  this->Actor->SetMapper(this->Mapper);

  this->StartFadeAngle = 30;
  this->EndFadeAngle = 20;
}

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::~TransformInteractionPipeline() = default;

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::InitializePipeline()
{
  this->CreateRotationHandles();
  this->CreateTranslationHandles();
  this->CreateScaleHandles();
  this->UpdateHandleVisibility();
  this->UpdateHandleColors();
}

//----------------------------------------------------------------------
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

//----------------------------------------------------------------------
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

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::CreateScaleHandles()
{
  this->AxisScaleHandleSource = vtkSmartPointer<vtkSphereSource>::New();
  this->AxisScaleHandleSource->SetRadius(INTERACTION_HANDLE_RADIUS);
  this->AxisScaleHandleSource->SetPhiResolution(16);
  this->AxisScaleHandleSource->SetThetaResolution(16);

  this->ScaleHandlePoints = vtkSmartPointer<vtkPolyData>::New();

  this->ScaleScaleTransform = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  this->ScaleScaleTransform->SetInputData(this->ScaleHandlePoints);
  this->ScaleScaleTransform->SetTransform(vtkNew<vtkTransform>());

  this->AxisScaleGlypher = vtkSmartPointer<vtkGlyph3D>::New();
  this->AxisScaleGlypher->SetInputConnection(this->ScaleScaleTransform->GetOutputPort());
  this->AxisScaleGlypher->SetSourceConnection(this->AxisScaleHandleSource->GetOutputPort());
  this->AxisScaleGlypher->ScalingOn();
  this->AxisScaleGlypher->SetScaleModeToDataScalingOff();
  this->AxisScaleGlypher->SetIndexModeToScalar();
  this->AxisScaleGlypher->SetColorModeToColorByScalar();

  vtkNew<vtkPoints> points; // Currently not enabled by default
  //points->InsertNextPoint(1.5, 0.0, 0.0); // X-axis
  //points->InsertNextPoint(0.0, 1.5, 0.0); // Y-axis
  //points->InsertNextPoint(0.0, 0.0, 1.5); // Z-axis
  this->ScaleHandlePoints->SetPoints(points);

  vtkNew<vtkIdTypeArray> visibilityArray;
  visibilityArray->SetName("visibility");
  visibilityArray->SetNumberOfComponents(1);
  visibilityArray->SetNumberOfValues(this->ScaleHandlePoints->GetNumberOfPoints());
  visibilityArray->Fill(1);
  this->ScaleHandlePoints->GetPointData()->AddArray(visibilityArray);

  this->Append->AddInputConnection(this->AxisScaleGlypher->GetOutputPort());
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::UpdateHandleVisibility()
{
  vtkSlicerLinearTransformWidgetRepresentation* TransformRepresentation = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->Representation);
  vtkMRMLTransformDisplayNode* displayNode = nullptr;
  if (TransformRepresentation)
    {
    displayNode = TransformRepresentation->GetTransformDisplayNode();
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

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::UpdateHandleColors()
{
  if (!this->ColorTable)
    {
    return;
    }

  int numberOfHandles = this->RotationHandlePoints->GetNumberOfPoints()
    + this->TranslationHandlePoints->GetNumberOfPoints()
    + this->ScaleHandlePoints->GetNumberOfPoints();
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

  // Rotation handles
  vtkSmartPointer<vtkFloatArray> scaleColorArray = vtkFloatArray::SafeDownCast(
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
    this->GetHandleColor(vtkMRMLTransformDisplayNode::ComponentScaleHandle, i, color);
    this->ColorTable->SetTableValue(colorIndex, color);
    scaleColorArray->SetTuple1(i, colorIndex);
    ++colorIndex;
    }

  this->ColorTable->Build();
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetHandleColor(int type, int index, double color[4])
{
  if (!color)
    {
    return;
    }

  double red[4]    = { 1.00, 0.00, 0.00, 1.00 };
  double green[4]  = { 0.00, 1.00, 0.00, 1.00 };
  double blue[4]   = { 0.00, 0.00, 1.00, 1.00 };
  double orange[4] = { 1.00, 0.50, 0.00, 1.00 };
  double white[4]  = { 1.00, 1.00, 1.00, 1.00 };
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

  vtkSlicerLinearTransformWidgetRepresentation* TransformRepresentation = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->Representation);
  vtkMRMLTransformDisplayNode* displayNode = nullptr;
  if (TransformRepresentation)
    {
    displayNode = TransformRepresentation->GetTransformDisplayNode();
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
  else if (type == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
    {
    handlePoints = this->ScaleHandlePoints;
    }

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

//----------------------------------------------------------------------
double vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetHandleOpacity(int type, int index)
{
  // Determine if the handle should be displayed
  bool handleVisible = true;
  vtkSlicerLinearTransformWidgetRepresentation* TransformRepresentation = vtkSlicerLinearTransformWidgetRepresentation::SafeDownCast(this->Representation);
  vtkMRMLTransformDisplayNode* displayNode = nullptr;
  if (TransformRepresentation)
    {
    displayNode = TransformRepresentation->GetTransformDisplayNode();
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

//----------------------------------------------------------------------
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

//----------------------------------------------------------------------
vtkSlicerLinearTransformWidgetRepresentation::HandleInfoList vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetHandleInfoList()
{
  vtkSlicerLinearTransformWidgetRepresentation::HandleInfoList handleInfoList;
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

  for (int i = 0; i < this->ScaleHandlePoints->GetNumberOfPoints(); ++i)
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
    }

  return handleInfoList;
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::SetWidgetScale(double scale)
{
  vtkNew<vtkTransform> scaleTransform;
  scaleTransform->Scale(scale, scale, scale);
  this->RotationScaleTransform->SetTransform(scaleTransform);
  this->TranslationScaleTransform->SetTransform(scaleTransform);
  this->ScaleScaleTransform->SetTransform(scaleTransform);
  this->AxisRotationGlypher->SetScaleFactor(scale);
  this->AxisTranslationGlypher->SetScaleFactor(scale);
  this->AxisScaleGlypher->SetScaleFactor(scale);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandleOriginWorld(double originWorld[3])
{
  if (!originWorld)
    {
    return;
    }

  double handleOrigin[3] = { 0,0,0 };
  this->HandleToWorldTransform->TransformPoint(handleOrigin, originWorld);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandleAxis(int type, int index, double axis[3])
{
  vtkPolyData* handles = nullptr;
  if (type == vtkMRMLTransformDisplayNode::ComponentRotationHandle)
    {
    handles = this->TranslationHandlePoints; // TODO
    }
  else if (type == vtkMRMLTransformDisplayNode::ComponentTranslationHandle)
    {
    handles = this->TranslationHandlePoints;
    }
  else if (type == vtkMRMLTransformDisplayNode::ComponentScaleHandle)
    {
    handles = this->ScaleHandlePoints;
    }

  if (!handles)
    {
    vtkErrorWithObjectMacro(nullptr, "GetInteractionHandleVector: Could not find interaction handles!");
    return;
    }

  if (index < 0 || index >= handles->GetNumberOfPoints())
    {
    vtkErrorWithObjectMacro(nullptr, "GetInteractionHandleVector: Handle index out of range!");
    return;
    }

  handles->GetPoint(index, axis);
}

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandleAxisWorld(int type, int index, double axisWorld[3])
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

//----------------------------------------------------------------------
void vtkSlicerLinearTransformWidgetRepresentation::TransformInteractionPipeline::GetInteractionHandlePositionWorld(int type, int index, double positionWorld[3])
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
    this->ScaleHandlePoints->GetPoint(index, positionWorld);
    this->HandleToWorldTransform->TransformPoint(positionWorld, positionWorld);
    }
}
