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

#ifndef __vtkMRMLLinearTransformsDisplayableManager3D_h
#define __vtkMRMLLinearTransformsDisplayableManager3D_h

// MRMLDisplayableManager includes
#include "vtkMRMLAbstractThreeDViewDisplayableManager.h"
#include "vtkSlicerLinearTransformWidget.h"
#include "vtkMRMLSliceNode.h"

#include "vtkSlicerTransformsModuleMRMLDisplayableManagerExport.h"

class vtkAbstractWidget;
class vtkMRMLTransformDisplayNode;


/// \brief Display transforms in 3D views
///
/// Displays transforms in 3D viewers as glyphs, deformed grid, or
/// contour surfaces
///
class VTK_SLICER_TRANSFORMS_MODULE_MRMLDISPLAYABLEMANAGER_EXPORT vtkMRMLLinearTransformsDisplayableManager3D
  : public vtkMRMLAbstractDisplayableManager
{
public:

  static vtkMRMLLinearTransformsDisplayableManager3D* New();
  vtkTypeMacro(vtkMRMLLinearTransformsDisplayableManager3D, vtkMRMLAbstractDisplayableManager);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  bool CanProcessInteractionEvent(vtkMRMLInteractionEventData* eventData, double& closestDistance2) override;
  bool ProcessInteractionEvent(vtkMRMLInteractionEventData* eventData) override;

  /// \internal
  /// For testing purposes only:
  /// Return the widget associated with the given transform, if any.
  //vtkAbstractWidget* GetWidget(vtkMRMLTransformDisplayNode* displayNode);

protected:

  vtkMRMLLinearTransformsDisplayableManager3D();
  ~vtkMRMLLinearTransformsDisplayableManager3D() override;

  vtkSlicerLinearTransformWidget* FindClosestWidget(vtkMRMLInteractionEventData* callData, double& closestDistance2);

  void UnobserveMRMLScene() override;
  void OnMRMLSceneNodeAdded(vtkMRMLNode* node) override;
  void OnMRMLSceneNodeRemoved(vtkMRMLNode* node) override;
  void ProcessMRMLNodesEvents(vtkObject* caller, unsigned long event, void* callData) override;

  /// Update Actors based on transforms in the scene
  void UpdateFromMRML() override;

  void OnMRMLSceneStartClose() override;
  void OnMRMLSceneEndClose() override;

  void OnMRMLSceneEndBatchProcess() override;
  /// Called after the corresponding MRML View container was modified
  void OnMRMLDisplayableNodeModifiedEvent(vtkObject* caller) override;
  virtual void OnMRMLSliceNodeModifiedEvent();

  /// Initialize the displayable manager
  void Create() override;

  void ProcessWidgetsEvents(vtkObject* caller, unsigned long event, void* callData) override;


  /// Accessor for internal flag that disables interactor style event processing
  vtkGetMacro(DisableInteractorStyleEventsProcessing, int);

  double LastClickWorldCoordinates[4];

  vtkWeakPointer<vtkSlicerLinearTransformWidget> LastActiveWidget;
private:

  vtkMRMLLinearTransformsDisplayableManager3D(const vtkMRMLLinearTransformsDisplayableManager3D&) = delete;
  void operator=(const vtkMRMLLinearTransformsDisplayableManager3D&) = delete;

  int DisableInteractorStyleEventsProcessing;


  class vtkInternal;
  vtkInternal* Internal;
  friend class vtkInternal;

  // by default, this displayableManager handles a 2d view, so the SliceNode
  // must be set when it's assigned to a viewer
  vtkWeakPointer<vtkMRMLSliceNode> SliceNode;
};

#endif
