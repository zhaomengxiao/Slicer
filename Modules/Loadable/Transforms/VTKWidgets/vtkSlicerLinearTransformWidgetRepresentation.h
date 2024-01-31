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
 * @class   vtkSlicerLinearTransformWidgetRepresentation
 * @brief   Class for rendering a Transform node
 *
 * This class can display a Transform node in the scene.
 * It plays a similar role to vtkWidgetRepresentation, but it is
 * simplified and specialized for optimal use in Slicer.
 * It state is stored in the associated MRML display node to
 * avoid extra synchronization mechanisms.
 * The representation only observes MRML node changes,
 * it does not directly process any interaction events directly
 * (interaction events are processed by vtkMRMLAbstractWidget,
 * which then modifies MRML nodes).
 *
 * This class (and subclasses) are a type of
 * vtkProp; meaning that they can be associated with a vtkRenderer end
 * embedded in a scene like any other vtkActor.
*
 * @sa
 * vtkSlicerTransformWidgetRepresentation vtkMRMLAbstractWidget vtkPointPlacer
*/

#ifndef vtkSlicerLinearTransformWidgetRepresentation_h
#define vtkSlicerLinearTransformWidgetRepresentation_h

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkArcSource.h>
#include <vtkArrowSource.h>
#include <vtkCellPicker.h>
#include <vtkGlyph3D.h>
#include <vtkLookupTable.h>
#include <vtkMRMLInteractionEventData.h>
#include <vtkPointPlacer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkTensorGlyph.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTubeFilter.h>

#include "vtkMRMLAbstractWidgetRepresentation.h"
#include "vtkMRMLTransformDisplayNode.h"
#include "vtkMRMLTransformNode.h"
#include "vtkSlicerTransformsModuleVTKWidgetsExport.h"




class VTK_SLICER_TRANSFORMS_MODULE_VTKWIDGETS_EXPORT vtkSlicerLinearTransformWidgetRepresentation : public vtkMRMLAbstractWidgetRepresentation
{
public:
	/// Instantiate this class.
	static vtkSlicerLinearTransformWidgetRepresentation* New();
  /// Standard methods for instances of this class.
  vtkTypeMacro(vtkSlicerLinearTransformWidgetRepresentation, vtkMRMLAbstractWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /// Update the representation from LinearTransform node
  void UpdateFromMRML(vtkMRMLNode* caller, unsigned long event, void *callData = nullptr) override;

  void UpdateFromMRMLInternal(vtkMRMLNode* caller, unsigned long event, void* callData = nullptr);

  /// Methods to make this class behave as a vtkProp.
  void GetActors(vtkPropCollection*) override;
  void ReleaseGraphicsResources(vtkWindow*) override;
  int RenderOverlay(vtkViewport* viewport) override;
  int RenderOpaqueGeometry(vtkViewport* viewport) override;
  int RenderTranslucentPolygonalGeometry(vtkViewport* viewport) override;
  vtkTypeBool HasTranslucentPolygonalGeometry() override;

  /// Set/Get the vtkMRMLTransformNode connected with this representation
  virtual void SetTransformDisplayNode(vtkMRMLTransformDisplayNode *transformDisplayNode);
  virtual vtkMRMLTransformDisplayNode* GetTransformDisplayNode();
  virtual vtkMRMLTransformNode* GetTransformNode();

  //for display, setup and update the composite actor
  void SetupPipline();
  void UpdatePipline();

  /// Get the axis for the handle specified by the index
  virtual void GetInteractionHandleAxisWorld(int type, int index, double axis[3]);
  void GetInteractionHandleOriginWorld(double originWorld[3]);
  

  //for interaction
  /// Return found component type (as vtkMRMLTransformDisplayNode::ComponentType).
  /// closestDistance2 is the squared distance in display coordinates from the closest position where interaction is possible.
  /// componentIndex returns index of the found component (e.g., if control point is found then control point index is returned).
  virtual void CanInteract(vtkMRMLInteractionEventData* interactionEventData,
    int& foundComponentType, int& foundComponentIndex, double& closestDistance2);

  /// Check if interaction with the transformation handles is possible
  virtual void CanInteractWithHandles(vtkMRMLInteractionEventData* interactionEventData,
    int& foundComponentType, int& foundComponentIndex, double& closestDistance2);

  bool AccuratePick(int x, int y, double pickPoint[3], double pickNormal[3] = nullptr);

  virtual vtkPointPlacer* GetPointPlacer();

protected:
  vtkSlicerLinearTransformWidgetRepresentation();
  ~vtkSlicerLinearTransformWidgetRepresentation() override;

  double GetViewScaleFactorAtPosition(double positionWorld[3], vtkMRMLInteractionEventData* interactionEventData = nullptr);

  virtual void SetTransformNode(vtkMRMLTransformNode *transformNode);

  class VTK_SLICER_TRANSFORMS_MODULE_VTKWIDGETS_EXPORT TransformInteractionPipeline
  {
  public:
    TransformInteractionPipeline(vtkMRMLAbstractWidgetRepresentation* representation);
    virtual ~TransformInteractionPipeline();
    
    vtkWeakPointer<vtkMRMLAbstractWidgetRepresentation> Representation;

    vtkSmartPointer<vtkSphereSource>                    AxisRotationHandleSource;
    vtkSmartPointer<vtkArcSource>                       AxisRotationArcSource;
    vtkSmartPointer<vtkTubeFilter>                      AxisRotationTubeFilter;
    vtkSmartPointer<vtkPolyData>                        AxisRotationInteriorAnglePolyData;
    vtkSmartPointer<vtkTubeFilter>                      AxisRotationInterorAngleTubeFilter;
    vtkSmartPointer<vtkPolyData>                        RotationHandlePoints;
    vtkSmartPointer<vtkTransformPolyDataFilter>         RotationScaleTransform;
    vtkSmartPointer<vtkAppendPolyData>                  AxisRotationGlyphSource;
    vtkSmartPointer<vtkTensorGlyph>                     AxisRotationGlypher;

    vtkSmartPointer<vtkArrowSource>                     AxisTranslationGlyphSource;
    vtkSmartPointer<vtkTransformPolyDataFilter>         AxisTranslationGlyphTransformer;
    vtkSmartPointer<vtkPolyData>                        TranslationHandlePoints;
    vtkSmartPointer<vtkTransformPolyDataFilter>         TranslationScaleTransform;
    vtkSmartPointer<vtkGlyph3D>                         AxisTranslationGlypher;

    /*vtkSmartPointer<vtkSphereSource>                    AxisScaleHandleSource;
    vtkSmartPointer<vtkPolyData>                        ScaleHandlePoints;
    vtkSmartPointer<vtkTransformPolyDataFilter>         ScaleScaleTransform;
    vtkSmartPointer<vtkGlyph3D>                         AxisScaleGlypher;*/

    vtkSmartPointer<vtkAppendPolyData>                  Append;
    vtkSmartPointer<vtkTransformPolyDataFilter>         HandleToWorldTransformFilter;
    vtkSmartPointer<vtkTransform>                       HandleToWorldTransform;
    vtkSmartPointer<vtkLookupTable>                     ColorTable;
    vtkSmartPointer<vtkPolyDataMapper>                  Mapper;
    vtkSmartPointer<vtkActor>                           Actor;
    vtkSmartPointer<vtkProperty>                        Property;

    double                                              StartFadeAngle{ 30 };
    double                                              EndFadeAngle{ 20 };
    double                                              InteractionHandleSize{ 1.0 };

    virtual void InitializePipeline();
    virtual void CreateRotationHandles();
    virtual void CreateTranslationHandles();
    virtual void UpdateHandleVisibility();
    virtual void UpdateHandleColors();
 
    /// Get the color of the specified handle
    /// Type is specified using vtkMRMLMarkupsDisplayNode::ComponentType
    virtual void GetHandleColor(int type, int index, double color[4]);
    /// Get the opacity of the specified handle
    virtual double GetHandleOpacity(int type, int index);
    /// Get the view plane normal for the widget in world coordinates
    virtual void GetViewPlaneNormal(double normal[3]);
    /// Get the direction vector of the interaction handle from the interaction origin in world coordinates
    void GetInteractionHandleAxisWorld(int type, int index, double axisWorld[3]);
    /// Get the interaction handle origin
    virtual void GetInteractionHandleOriginWorld(double originWorld[3]);
    struct HandleInfo
    {
      HandleInfo(int index, int componentType, double positionWorld[3], double positionLocal[3], double color[4])
        : Index(index)
        , ComponentType(componentType)
      {
        for (int i = 0; i < 3; ++i)
        {
          this->PositionWorld[i] = positionWorld[i];
        }
        this->PositionWorld[3] = 1.0;
        for (int i = 0; i < 3; ++i)
        {
          this->PositionLocal[i] = positionLocal[i];
        }
        this->PositionLocal[3] = 1.0;
        for (int i = 0; i < 4; ++i)
        {
          this->Color[i] = color[i];
        }
      }
      int Index;
      int ComponentType;
      double PositionLocal[4];
      double PositionWorld[4];
      double Color[4];
      bool IsVisible()
      {
        double epsilon = 0.001;
        return this->Color[3] > epsilon;
      }
    };

    /// Get the list of info for all interaction handles
    virtual std::vector<HandleInfo> GetHandleInfoList();
  };
  typedef std::vector<TransformInteractionPipeline::HandleInfo> HandleInfoList;

  vtkWeakPointer<vtkMRMLTransformDisplayNode> TransformDisplayNode;
  vtkWeakPointer<vtkMRMLTransformNode> TransformNode;

  vtkTimeStamp TransformTransformModifiedTime;

  TransformInteractionPipeline* InteractionPipeline;

  vtkSmartPointer<vtkPointPlacer> PointPlacer;

  vtkSmartPointer<vtkCellPicker> AccuratePicker;

private:
	vtkSlicerLinearTransformWidgetRepresentation(const vtkSlicerLinearTransformWidgetRepresentation&) = delete;
  void operator=(const vtkSlicerLinearTransformWidgetRepresentation&) = delete;
};

#endif
