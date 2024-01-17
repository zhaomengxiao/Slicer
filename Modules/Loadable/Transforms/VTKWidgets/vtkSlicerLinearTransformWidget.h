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
 * @class   vtkSlicerLinearTransformWidget
 * @brief   Process interaction events to update state of markup widget nodes
 *
 * @sa
 * vtkMRMLAbstractWidget vtkSlicerWidgetRepresentation vtkSlicerWidgetEventTranslator
 *
*/

#ifndef vtkSlicerLinearTransformWidget_h
#define vtkSlicerLinearTransformWidget_h

#include "vtkSlicerTransformsModuleVTKWidgetsExport.h"
#include "vtkMRMLAbstractWidget.h"
#include "vtkMRMLSelectionNode.h"
#include "vtkWidgetCallbackMapper.h"

#include "vtkMRMLTransformNode.h"
#include "vtkMRMLTransformDisplayNode.h"

class vtkMRMLAbstractViewNode;
class vtkMRMLApplicationLogic;
class vtkMRMLInteractionEventData;
class vtkMRMLInteractionNode;
class vtkIdList;
class vtkPolyData;
class vtkSlicerLinearTransformWidgetRepresentation;

class VTK_SLICER_TRANSFORMS_MODULE_VTKWIDGETS_EXPORT vtkSlicerLinearTransformWidget : public vtkMRMLAbstractWidget
{
public:
  /// Standard methods for a VTK class.
  vtkTypeMacro(vtkSlicerLinearTransformWidget, vtkMRMLAbstractWidget);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /// Create the default widget representation and initializes the widget and representation.
  virtual void CreateDefaultRepresentation(vtkMRMLTransformDisplayNode* markupsDisplayNode, vtkMRMLAbstractViewNode* viewNode, vtkRenderer* renderer) = 0;

  /// Create instance of the Transform widget
  virtual vtkSlicerLinearTransformWidget* CreateInstance() const = 0;

  /// Widget states
  enum
  {
    WidgetStateOnNothing = WidgetStateUser, // click in empty area no interaction
    WidgetStateOnTranslationHandle, // hovering over a translation interaction handle
    WidgetStateOnRotationHandle, // hovering over a rotation interaction handle
    WidgetStateOnScaleHandle, // hovering over a scale interaction handle
    WidgetStateTransform_Last
  };

  /// Widget events
  enum
  {
    WidgetEventClickAndDragStart = WidgetEventUser,
    WidgetEventClickAndDragEnd,
    WidgetEventReserved,  // this events is only to prevent other widgets from processing an event
    WidgetEventTransform_Last
  };

  /// Return true if the widget can process the event.
  bool CanProcessInteractionEvent(vtkMRMLInteractionEventData* eventData, double &distance2) override;

  /// Process interaction event.
  bool ProcessInteractionEvent(vtkMRMLInteractionEventData* eventData) override;

  /// Called when the the widget loses the focus.
  void Leave(vtkMRMLInteractionEventData* eventData) override;

  // Allows the widget to request interactive mode (faster updates)
  bool GetInteractive() override;
  // Allows the widget to request a cursor shape
  int GetMouseCursor() override;

  vtkMRMLTransformNode* GetTransformNode();
  vtkMRMLTransformDisplayNode* GetTransformDisplayNode();

  vtkSlicerLinearTransformWidgetRepresentation* GetTransformRepresentation();

  int GetActiveComponentType();
  int GetActiveComponentIndex();
  vtkMRMLSelectionNode* selectionNode();

protected:
  vtkSlicerLinearTransformWidget();
  ~vtkSlicerLinearTransformWidget() override;

  void StartWidgetInteraction(vtkMRMLInteractionEventData* eventData);
  void EndWidgetInteraction();

  virtual void TranslateWidget(double eventPos[2]);
  virtual void ScaleWidget(double eventPos[2]);
  virtual void RotateWidget(double eventPos[2]);

  // Get accurate world position.
  // World position that comes in the event data may be inaccurate, this method computes a more reliable position.
  // Returns true on success.
  // refWorldPos is an optional reference position: if point distance from camera cannot be determined then
  // depth of this reference position is used.
  bool ConvertDisplayPositionToWorld(const int displayPos[2], double worldPos[3], double worldOrientationMatrix[9],
    double* refWorldPos = nullptr);


  // Callback interface to capture events when
  // placing the widget.
  // Return true if the event is processed.
  virtual bool ProcessMouseMove(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessWidgetMenu(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessWidgetAction(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessWidgetTranslateStart(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessWidgetRotateStart(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessWidgetScaleStart(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessEndMouseDrag(vtkMRMLInteractionEventData* eventData);
  virtual bool ProcessWidgetJumpCursor(vtkMRMLInteractionEventData* eventData);

  // Get the closest point on the line defined by the interaction handle axis.
  // Input coordinates are in display coordinates, while output are in world coordinates.
  virtual bool GetClosestPointOnInteractionAxis(int type, int index, const double inputDisplay[2], double outputIntersectionWorld[3]);

  // Get the closest point on the plane defined using the interaction handle axis as the plane normal.
  // Input coordinates are in display coordinates, while output are in world coordinates
  virtual bool GetIntersectionOnAxisPlane(int type, int index, const double inputDisplay[2], double outputIntersectionWorld[3]);

  // Variables for translate/rotate/scale
  double LastEventPosition[2];
  double StartEventOffsetPosition[2];

private:
  vtkSlicerLinearTransformWidget(const vtkSlicerLinearTransformWidget&) = delete;
  void operator=(const vtkSlicerLinearTransformWidget&) = delete;
};

//----------------------------------------------------------------------
// CREATE INSTANCE MACRO

#ifdef VTK_HAS_INITIALIZE_OBJECT_BASE
#define vtkSlicerLinearTransformWidgetCreateInstanceMacro(type) \
vtkSlicerLinearTransformWidget* CreateInstance() const override\
{ \
  vtkObject* ret = vtkObjectFactory::CreateInstance(#type); \
  if(ret) \
    { \
    return static_cast<type *>(ret); \
    } \
  type* result = new type; \
  result->InitializeObjectBase(); \
  return result; \
}
#else
#define vtkSlicerLinearTransformWidgetCreateInstanceMacro(type) \
vtkSlicerLinearTransformWidget* CreateInstance() const override\
{ \
  vtkObject* ret = vtkObjectFactory::CreateInstance(#type); \
  if(ret) \
    { \
    return static_cast<type *>(ret); \
    } \
  return new type; \
}
#endif

#endif
