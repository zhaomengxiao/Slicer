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

/**
 * @class   vtkSlicerLinearTransformWidgetRepresentation2D
 * @brief   Default representation for the slicer transform widget
 *
 * This class provides the default concrete representation for the
 * vtkMRMLAbstractWidget. See vtkMRMLAbstractWidget
 * for details.
 * @sa
 * vtkSlicerLinearTransformWidgetRepresentation2D vtkMRMLAbstractWidget
*/

#ifndef vtkSlicerLinearTransformWidgetRepresentation2D_h
#define vtkSlicerLinearTransformWidgetRepresentation2D_h

#include "vtkSlicerTransformsModuleVTKWidgetsExport.h"
#include "vtkSlicerLinearTransformWidgetRepresentation.h"

#include "vtkMRMLSliceNode.h"

class vtkActor2D;
class vtkDiscretizableColorTransferFunction;
class vtkGlyph2D;
class vtkLabelPlacementMapper;
class vtkTransformGlyphSource2D;
class vtkPlane;
class vtkPolyDataMapper2D;
class vtkProperty2D;

class vtkMRMLInteractionEventData;

class VTK_SLICER_TRANSFORMS_MODULE_VTKWIDGETS_EXPORT vtkSlicerLinearTransformWidgetRepresentation2D : public vtkSlicerLinearTransformWidgetRepresentation
{
public:
  /// Instantiate this class.
  static vtkSlicerLinearTransformWidgetRepresentation2D* New();

  /// Standard methods for instances of this class.
  vtkTypeMacro(vtkSlicerLinearTransformWidgetRepresentation2D, vtkSlicerLinearTransformWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /// Position is displayed (slice) position
  void CanInteract(vtkMRMLInteractionEventData* interactionEventData,
    int &foundComponentType, int &foundComponentIndex, double &closestDistance2) override;

  /// Check if interaction with the transformation handles is possible
  virtual void CanInteractWithHandles(vtkMRMLInteractionEventData* interactionEventData,
    int& foundComponentType, int& foundComponentIndex, double& closestDistance2);

  /// Subclasses of vtkSlicerLinearTransformWidgetRepresentation2D must implement these methods. These
  /// are the methods that the widget and its representation use to
  /// communicate with each other.
  void UpdateFromMRMLInternal(vtkMRMLNode* caller, unsigned long event, void *callData=nullptr) override;

  /// Methods to make this class behave as a vtkProp.
  void GetActors(vtkPropCollection *) override;
  void ReleaseGraphicsResources(vtkWindow *) override;
  int RenderOverlay(vtkViewport *viewport) override;
  int RenderOpaqueGeometry(vtkViewport *viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport *viewport) override;
  vtkTypeBool HasTranslucentPolygonalGeometry() override;

  /// Set the center slice visibility (i.e. if it is on the slice).
  virtual void SetCenterSliceVisibility(bool visibility);

  void GetSliceToWorldCoordinates(const double[2], double[3]);
  void GetWorldToSliceCoordinates(const double worldPos[3], double slicePos[2]);

  void UpdateInteractionPipeline() override;

protected:
  vtkSlicerLinearTransformWidgetRepresentation2D();
  ~vtkSlicerLinearTransformWidgetRepresentation2D() override;

  /// Reimplemented for 2D specific mapper/actor settings
  void SetupInteractionPipeline() override;

    /// Get MRML view node as slice view node
  vtkMRMLSliceNode *GetSliceNode();

  void UpdatePlaneFromSliceNode();

  void UpdateViewScaleFactor() override;

  // Return squared distance of maximum distance for picking an interaction handle,
  // in pixels.
  double GetMaximumInteractionHandlePickingDistance2();

  /// Convert display to world coordinates
  void GetWorldToDisplayCoordinates(double r, double a, double s, double * displayCoordinates);
  void GetWorldToDisplayCoordinates(double * worldCoordinates, double * displayCoordinates);

  /// Check if the representation polydata intersects the slice
  bool IsRepresentationIntersectingSlice(vtkPolyData* representation, const char* arrayName);

  vtkSmartPointer<vtkIntArray> PointsVisibilityOnSlice;
  bool                         CenterVisibilityOnSlice = { false };
  bool                         AnyPointVisibilityOnSlice = { false };  // at least one point is visible

  vtkSmartPointer<vtkTransform> WorldToSliceTransform;
  vtkSmartPointer<vtkPlane> SlicePlane;

  class TransformInteractionPipeline2D : public TransformInteractionPipeline
  {
  public:
    TransformInteractionPipeline2D(vtkSlicerLinearTransformWidgetRepresentation* representation);
    ~TransformInteractionPipeline2D() override = default;;

    void GetViewPlaneNormal(double viewPlaneNormal[3]) override;

    vtkSmartPointer<vtkTransformPolyDataFilter> WorldToSliceTransformFilter;
  };

private:
  vtkSlicerLinearTransformWidgetRepresentation2D(const vtkSlicerLinearTransformWidgetRepresentation2D&) = delete;
  void operator=(const vtkSlicerLinearTransformWidgetRepresentation2D&) = delete;
};

#endif
