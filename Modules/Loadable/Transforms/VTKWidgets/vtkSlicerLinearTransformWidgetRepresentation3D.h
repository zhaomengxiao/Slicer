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
 * @class   vtkSlicerLinearTransformWidgetRepresentation3D
 * @brief   Default representation for the transform widget in 3D views
 *
 * @sa
 * vtkSlicerLinearTransformWidgetRepresentation vtkSlicerTransformWidget
*/

#ifndef vtkSlicerLinearTransformWidgetRepresentation3D_h
#define vtkSlicerLinearTransformWidgetRepresentation3D_h

#include "vtkSlicerTransformsModuleVTKWidgetsExport.h"
#include "vtkSlicerLinearTransformWidgetRepresentation.h"

#include <map>

class vtkActor;
class vtkActor2D;
class vtkCellPicker;
class vtkFastSelectVisiblePoints;
class vtkGlyph3DMapper;
class vtkLabelPlacementMapper;
class vtkPolyDataMapper;
class vtkProperty;

class vtkMRMLInteractionEventData;

class VTK_SLICER_TRANSFORMS_MODULE_VTKWIDGETS_EXPORT vtkSlicerLinearTransformWidgetRepresentation3D : public vtkSlicerLinearTransformWidgetRepresentation
{
public:
  /// Instantiate this class.
  static vtkSlicerLinearTransformWidgetRepresentation3D* New();

  /// Standard methods for instances of this class.
  vtkTypeMacro(vtkSlicerLinearTransformWidgetRepresentation3D, vtkSlicerLinearTransformWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /// Subclasses of vtkSlicerLinearTransformWidgetRepresentation3D must implement these methods. These
  /// are the methods that the widget and its representation use to
  /// communicate with each other.
  void UpdateFromMRMLInternal(vtkMRMLNode* caller, unsigned long event, void *callData = nullptr) override;

  /// Methods to make this class behave as a vtkProp.
  void GetActors(vtkPropCollection *) override;
  void ReleaseGraphicsResources(vtkWindow *) override;
  int RenderOverlay(vtkViewport *viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport *viewport) override;
  vtkTypeBool HasTranslucentPolygonalGeometry() override;

  /// Return the bounds of the representation
  double* GetBounds() VTK_SIZEHINT(6) override;

  void CanInteract(vtkMRMLInteractionEventData* interactionEventData,
    int &foundComponentType, int &foundComponentIndex, double &closestDistance2) override;

  /// Check if interaction with the transformation handles is possible
  virtual void CanInteractWithHandles(vtkMRMLInteractionEventData* interactionEventData,
    int& foundComponentType, int& foundComponentIndex, double& closestDistance2);

  bool AccuratePick(int x, int y, double pickPoint[3], double pickNormal[3]=nullptr);

  /// Relative offset used for rendering occluded actors.
  /// The range of coincident offset can be between +/- 65000.
  /// Positive values move the occluded objects away from the camera, and negative values towards.
  /// Default value is -25000.
  vtkSetMacro(OccludedRelativeOffset, double);
  vtkGetMacro(OccludedRelativeOffset, double);

protected:
  vtkSlicerLinearTransformWidgetRepresentation3D();
  ~vtkSlicerLinearTransformWidgetRepresentation3D() override;

  double GetViewScaleFactorAtPosition(double positionWorld[3], vtkMRMLInteractionEventData* interactionEventData = nullptr);

  void UpdateViewScaleFactor() override;

  void UpdateInteractionPipeline() override;

  /// Update the occluded relative offsets for an occluded mapper
  /// Allows occluded regions to be rendered on top.
  /// Sets the following parameter on the mappers:
  /// - RelativeCoincidentTopologyLineOffsetParameters
  /// - RelativeCoincidentTopologyPolygonOffsetParameters
  /// - RelativeCoincidentTopologyPointOffsetParameter
  void UpdateRelativeCoincidentTopologyOffsets(vtkMapper* mapper, vtkMapper* occludedMapper);
  using vtkMRMLAbstractWidgetRepresentation::UpdateRelativeCoincidentTopologyOffsets;

  vtkSmartPointer<vtkCellPicker> AccuratePicker;

  double TextActorPositionWorld[3];
  bool TextActorOccluded;
  double OccludedRelativeOffset;

private:
  vtkSlicerLinearTransformWidgetRepresentation3D(const vtkSlicerLinearTransformWidgetRepresentation3D&) = delete;
  void operator=(const vtkSlicerLinearTransformWidgetRepresentation3D&) = delete;
};

#endif
