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
 * vtkSlicerMarkupsWidgetRepresentation vtkMRMLAbstractWidget vtkPointPlacer
*/

#ifndef vtkSlicerLinearTransformWidgetRepresentation_h
#define vtkSlicerLinearTransformWidgetRepresentation_h

#include "vtkSlicerTransformsModuleVTKWidgetsExport.h"

#include "vtkMRMLAbstractWidgetRepresentation.h"
#include "vtkMRMLTransformDisplayNode.h"
#include "vtkMRMLTransformNode.h"

#include "vtkActor2D.h"
#include "vtkAppendPolyData.h"
#include "vtkArcSource.h"
#include "vtkArrowSource.h"
#include "vtkGlyph3D.h"
#include "vtkLookupTable.h"
#include "vtkPointPlacer.h"
#include "vtkPolyDataMapper2D.h"
#include "vtkProperty2D.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkTensorGlyph.h"
#include "vtkTextActor.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkTubeFilter.h"

class vtkMRMLInteractionEventData;

class VTK_SLICER_TRANSFORMS_MODULE_VTKWIDGETS_EXPORT vtkSlicerLinearTransformWidgetRepresentation : public vtkMRMLAbstractWidgetRepresentation
{
public:
  /// Standard methods for instances of this class.
  vtkTypeMacro(vtkSlicerLinearTransformWidgetRepresentation, vtkMRMLAbstractWidgetRepresentation);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /// Update the representation from LinearTransform node
  void UpdateFromMRML(vtkMRMLNode* caller, unsigned long event, void *callData = nullptr) override;
  virtual void UpdateFromMRMLInternal(vtkMRMLNode* caller, unsigned long event, void *callData = nullptr);

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

  /// Compute the center of rotation and update it in the Transform node.
  //virtual void UpdateCenterOfRotation();

  /// Translation, rotation, scaling will happen around this position
  virtual bool GetTransformationReferencePoint(double referencePointWorld[3]);

  /// Return found component type (as vtkMRMLTransformDisplayNode::ComponentType).
  /// closestDistance2 is the squared distance in display coordinates from the closest position where interaction is possible.
  /// componentIndex returns index of the found component (e.g., if control point is found then control point index is returned).
  virtual void CanInteract(vtkMRMLInteractionEventData* interactionEventData,
	int &foundComponentType, int &foundComponentIndex, double &closestDistance2);

	virtual vtkPointPlacer* GetPointPlacer();
  //@{
  /**
  * Returns true if the representation is displayable in the current view.
  * It takes into account current view node's display node and parent folder's visibility.
  */
  bool IsDisplayable();
  //@}

  /// Get the axis for the handle specified by the index
  virtual void GetInteractionHandleAxisWorld(int type, int index, double axis[3]);
  /// Get the origin of the interaction handle widget
  virtual void GetInteractionHandleOriginWorld(double origin[3]);
  /// Get the position of an interaction handle in world coordinates
  virtual void GetInteractionHandlePositionWorld(int type, int index, double position[3]);

	/// Update the interaction pipeline
	virtual void UpdateInteractionPipeline();

protected:
  vtkSlicerLinearTransformWidgetRepresentation();
  ~vtkSlicerLinearTransformWidgetRepresentation() override;


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

	vtkSmartPointer<vtkSphereSource>                    AxisScaleHandleSource;
	vtkSmartPointer<vtkPolyData>                        ScaleHandlePoints;
	vtkSmartPointer<vtkTransformPolyDataFilter>         ScaleScaleTransform;
	vtkSmartPointer<vtkGlyph3D>                         AxisScaleGlypher;

	vtkSmartPointer<vtkAppendPolyData>                  Append;
	vtkSmartPointer<vtkTransformPolyDataFilter>         HandleToWorldTransformFilter;
	vtkSmartPointer<vtkTransform>                       HandleToWorldTransform;
	vtkSmartPointer<vtkLookupTable>                     ColorTable;
	vtkSmartPointer<vtkPolyDataMapper2D>                Mapper;
	vtkSmartPointer<vtkActor2D>                         Actor;
	vtkSmartPointer<vtkProperty2D>                      Property;

	double                                              StartFadeAngle{30};
	double                                              EndFadeAngle{20};
	double                                              InteractionHandleSize{1.0};

	virtual void InitializePipeline();
	virtual void CreateRotationHandles();
	virtual void CreateTranslationHandles();
	virtual void CreateScaleHandles();
	virtual void UpdateHandleVisibility();
	virtual void UpdateHandleColors();

	/// Set the scale of the interaction handles in world coordinates
	virtual void SetWidgetScale(double scale);
	/// Get the color of the specified handle
	/// Type is specified using vtkMRMLTransformDisplayNode::ComponentType
	virtual void GetHandleColor(int type, int index, double color[4]);
	/// Get the opacity of the specified handle
	virtual double GetHandleOpacity(int type, int index);

	/// Get the view plane normal for the widget in world coordinates
	virtual void GetViewPlaneNormal(double normal[3]);

	/// Get the position of the interaction handle in world coordinates
	/// Type is specified using vtkMRMLTransformDisplayNode::ComponentType
	virtual void GetInteractionHandlePositionWorld(int type, int index, double position[3]);
	/// Get the direction vector of the interaction handle from the interaction origin
	/// Type is specified using vtkMRMLTransformDisplayNode::ComponentType
	virtual void GetInteractionHandleAxis(int type, int index, double axis[3]);
	/// Get the direction vector of the interaction handle from the interaction origin in world coordinates
	virtual void GetInteractionHandleAxisWorld(int type, int index, double axis[3]);
	/// Get the interaction handle origin
	virtual void GetInteractionHandleOriginWorld(double origin[3]);

/*#if !defined(__VTK_WRAP__)*/
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
	/*#endif*/
	/// Get the list of info for all interaction handles
	virtual std::vector<HandleInfo> GetHandleInfoList();
  };
  typedef std::vector<TransformInteractionPipeline::HandleInfo> HandleInfoList;

  // Calculate view size and scale factor
  virtual void UpdateViewScaleFactor() = 0;

  // Update the size of the interaction handle based on screen size + vtkMRMLTransformDisplayNode::InteractionHandleScale parameter.
  virtual void UpdateInteractionHandleSize();

  double ViewScaleFactorMmPerPixel;
  double ScreenSizePixel; // diagonal size of the screen

  virtual void SetTransformNode(vtkMRMLTransformNode *transformNode);

  vtkWeakPointer<vtkMRMLTransformDisplayNode> TransformDisplayNode;
  vtkWeakPointer<vtkMRMLTransformNode> TransformNode;

  vtkSmartPointer<vtkPointPlacer> PointPlacer;

  vtkSmartPointer<vtkTextActor> TextActor;

  vtkTypeBool CurveClosed;

  vtkTimeStamp TransformTransformModifiedTime;

  virtual void SetupInteractionPipeline();
  TransformInteractionPipeline* InteractionPipeline;

  

private:
	vtkSlicerLinearTransformWidgetRepresentation(const vtkSlicerLinearTransformWidgetRepresentation&) = delete;
  void operator=(const vtkSlicerLinearTransformWidgetRepresentation&) = delete;
};

#endif
