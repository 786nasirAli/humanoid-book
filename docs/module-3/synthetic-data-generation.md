---
sidebar_position: 5
---

# Synthetic Data Generation

Synthetic data generation is crucial for training robust AI models for humanoid robots. Using Isaac Sim's capabilities, we can create diverse, annotated datasets that would be difficult or expensive to collect in the real world.

## Benefits of Synthetic Data

- **Cost-Effective**: No need for physical data collection
- **Safety**: Risk-free environment for testing extreme scenarios
- **Variety**: Easy to generate diverse scenarios and conditions
- **Ground Truth**: Perfect annotations available for all data
- **Reproducibility**: Exactly repeatable experiments and tests
- **Scalability**: Generate large datasets quickly

## Isaac Sim Data Generation Pipeline

### Scene Randomization

Isaac Sim enables extensive scene randomization:

- **Lighting Conditions**: Dynamic lighting with various intensities and angles
- **Material Properties**: Randomized textures, colors, and surface properties
- **Object Placement**: Stochastic object arrangements
- **Weather Simulation**: Different environmental conditions
- **Camera Parameters**: Randomized intrinsic and extrinsic camera parameters

### Annotation Tools

Isaac Sim provides several annotation types:

- **Bounding Boxes**: 2D and 3D bounding boxes for object detection
- **Segmentation Masks**: Instance and semantic segmentation
- **Keypoint Annotation**: For humanoid joint detection
- **Depth Maps**: Ground truth depth information
- **Pose Estimation**: 6-DOF pose annotations

## Domain Randomization

To bridge the reality gap, we implement domain randomization techniques:

- **Texture Randomization**: Random textures on objects
- **Color Variation**: Stochastic color schemes
- **Geometric Variations**: Random scales and positions
- **Atmospheric Effects**: Randomized fog and atmospheric conditions
- **Lighting Diversity**: Extensive lighting condition variations

## Synthetic Data Pipeline Configuration

The pipeline is configured through YAML files that specify:

- **Scene Parameters**: Environment settings and randomization
- **Sensor Settings**: Camera, LIDAR, and IMU configurations
- **Annotation Types**: What annotations to generate
- **Output Formats**: How to store and format the data
- **Performance Settings**: Optimization parameters

## Data Quality Assurance

- **Validation Checks**: Ensuring generated data meets quality standards
- **Diversity Metrics**: Measuring how diverse the generated dataset is
- **Realism Assessment**: Comparing synthetic data to real-world data
- **Consistency Verification**: Ensuring annotations are accurate

## Integration with Training Workflows

- **Format Compatibility**: Ensuring datasets are compatible with training frameworks
- **Pipeline Efficiency**: Optimizing for high-throughput data generation
- **Storage Management**: Efficient storage and retrieval of large datasets
- **Data Augmentation**: Integration with additional augmentation techniques

## Practical Assignment - Building Synthetic Data Pipeline

### Assignment 1: Basic Data Generation Pipeline

**Objective**: Create a simple synthetic data generation pipeline for humanoid perception tasks.

**Detailed Steps**:
1. **Set Up Isaac Sim Environment**:
   - Create a simple indoor environment in Isaac Sim
   - Add a humanoid robot with RGB camera sensor
   - Set up basic objects for the robot to perceive

2. **Configure Data Capture**:
   ```python
   # Create a basic data capture script
   import omni
   from omni.isaac.synthetic_utils import SyntheticDataExtractor

   # Initialize the synthetic data extractor
   extractor = SyntheticDataExtractor(
       "/Isaac/Editor/Viewport",
       "/Isaac/Sensors/RgbdCamera",
       "/World/Robot/base_link"
   )

   # Define capture parameters
   capture_settings = {
       "color": True,
       "depth": True,
       "instance_segmentation": True,
       "bounding_boxes": True,
       "frequency": 30,  # Hz
   }
   ```

3. **Create Simple Object Detection Scene**:
   - Add 5-10 objects of different shapes and colors
   - Position them at various distances from the robot
   - Ensure they're all within the camera's field of view

4. **Run Data Generation**:
   ```bash
   # Use Isaac Sim's data generation tools
   python -m omni.isaac.synthetic_utils.capture_isaac_data \
     --scene_path "/path/to/your/scene.usd" \
     --output_dir "./synthetic_dataset" \
     --num_frames 1000 \
     --capture_settings config/capture_settings.yaml
   ```

5. **Verify Generated Data**:
   - Check that RGB images were captured
   - Verify depth maps and segmentation masks
   - Ensure bounding box annotations are correct
   - Validate file structure and naming convention

**Expected Outcome**: A dataset of 1000 RGB-D frames with ground truth annotations.

**Code Example for Data Verification**:
```python
# Python script to verify generated data
import os
import cv2
import numpy as np
import json

def verify_dataset(dataset_path):
    # Check directory structure
    required_dirs = ['color', 'depth', 'segmentation', 'labels']
    for dir_name in required_dirs:
        if not os.path.exists(os.path.join(dataset_path, dir_name)):
            print(f"Missing directory: {dir_name}")
            return False

    # Check number of files
    color_files = os.listdir(os.path.join(dataset_path, 'color'))
    depth_files = os.listdir(os.path.join(dataset_path, 'depth'))

    if len(color_files) != len(depth_files):
        print("Mismatched number of color and depth files")
        return False

    print(f"Dataset verified: {len(color_files)} frames")

    # Sample a few files to verify content
    for i in range(min(5, len(color_files))):
        color_path = os.path.join(dataset_path, 'color', color_files[i])
        img = cv2.imread(color_path)
        if img is None:
            print(f"Invalid image: {color_path}")
            return False

    return True

# Verify the generated dataset
dataset_path = "./synthetic_dataset"
is_valid = verify_dataset(dataset_path)
print(f"Dataset is valid: {is_valid}")
```

**Learning Points**:
- Understanding Isaac Sim's synthetic data capabilities
- Learning to set up simple data capture scenarios
- Recognizing the importance of data validation

### Assignment 2: Domain Randomization Implementation

**Objective**: Implement domain randomization techniques to improve dataset diversity.

**Detailed Steps**:
1. **Create Material Randomization Script**:
   ```python
   # material_randomizer.py
   import random
   import omni
   from pxr import Gf, Sdf, UsdShade
   from omni.isaac.core.utils.materials import create_diffuse_color_material

   # Define material properties range
   material_config = {
       "roughness_range": (0.1, 0.9),
       "metallic_range": (0.0, 0.3),
       "color_variance": 0.2
   }

   def randomize_materials(stage, root_path="/World/Objects"):
       """Randomize materials in the scene"""
       for prim in stage.Traverse():
           if not prim.GetTypeName() == "Mesh":
               continue

           # Skip the robot to maintain consistency
           if "Robot" in prim.GetPath().pathString:
               continue

           # Randomize material properties
           roughness = random.uniform(*material_config["roughness_range"])
           metallic = random.uniform(*material_config["metallic_range"])

           # Create a random color with some variance
           base_color = Gf.Vec3f(
               random.uniform(0.3, 0.9),
               random.uniform(0.3, 0.9),
               random.uniform(0.3, 0.9)
           )

           # Apply the random material
           material_path = f"{prim.GetPath()}_Material"
           material = create_diffuse_color_material(material_path, base_color)

           # Apply material to mesh
           UsdShade.MaterialBindingAPI(prim).Bind(material)

   # Apply material randomization
   stage = omni.usd.get_context().get_stage()
   randomize_materials(stage)
   ```

2. **Implement Lighting Randomization**:
   ```python
   # lighting_randomizer.py
   import random
   from pxr import Gf, UsdLux, Sdf

   def randomize_lighting(stage):
       """Randomize lighting conditions in the scene"""
       # Find the main light in the scene
       light_path = Sdf.Path("/World/DistantLight")
       light_prim = stage.GetPrimAtPath(light_path)

       if light_prim and light_prim.IsValid():
           distant_light = UsdLux.DistantLight(light_prim)

           # Random intensity
           intensity = random.uniform(500, 3000)
           distant_light.CreateIntensityAttr(intensity)

           # Random color temperature (affects light color)
           color_temp = random.uniform(4000, 8000)  # Kelvin
           # Rough approximation of color from temperature
           if color_temp < 6000:
               color = Gf.Vec3f(1.0, 0.9, 0.8)  # Warmer
           else:
               color = Gf.Vec3f(0.9, 0.95, 1.0)  # Cooler

           distant_light.CreateColorAttr(color)

           # Random direction
           direction = Gf.Vec3f(
               random.uniform(-1, 1),
               random.uniform(-1, 1),
               random.uniform(-1, 0)  # Mostly downward
           )
           distant_light.AddTranslateOp().Set(direction * 5)  # Move light
   ```

3. **Create Scene Layout Randomization**:
   ```python
   # scene_randomizer.py
   import random
   from pxr import Gf, UsdGeom
   import omni

   def randomize_scene_layout(stage, num_objects=10):
       """Randomize the positions of objects in the scene"""
       # Get all object prims
       object_prims = []
       for prim in stage.Traverse():
           if prim.GetTypeName() == "Mesh" and "Robot" not in prim.GetPath().pathString:
               object_prims.append(prim)

       # Randomize positions within a bounded area
       for i, prim in enumerate(object_prims):
           if i >= num_objects:  # Only randomize the specified number
               break

           x = random.uniform(-2.0, 2.0)
           y = random.uniform(-2.0, 2.0)
           z = random.uniform(0.1, 0.5)  # Slightly above ground

           # Get the Xformable prim
           xformable = UsdGeom.Xformable(prim)
           if xformable:
               xformable.AddTranslateOp().Set(Gf.Vec3f(x, y, z))

   # Apply scene randomization
   stage = omni.usd.get_context().get_stage()
   randomize_scene_layout(stage)
   ```

4. **Create Randomization Workflow**:
   ```python
   # complete_randomization_pipeline.py
   import omni
   import carb
   from pxr import Gf, UsdGeom

   def run_randomization_pipeline(num_samples=100):
       """Run the complete randomization pipeline"""
       stage = omni.usd.get_context().get_stage()

       for i in range(num_samples):
           print(f"Generating sample {i+1}/{num_samples}")

           # Apply randomizations
           randomize_materials(stage)
           randomize_lighting(stage)
           randomize_scene_layout(stage)

           # Simulate briefly to settle objects
           for _ in range(10):
               omni.timeline.get_timeline_interface().update_current_time(carb.float64(1.0/60.0))

           # Capture the frame
           # (Implementation would depend on Isaac Sim's recording API)

           print(f"Sample {i+1} captured")

   # Example usage
   # run_randomization_pipeline(50)  # Generate 50 randomized frames
   ```

5. **Execute the Randomization Pipeline**:
   ```bash
   # Run the domain randomization pipeline
   python scripts/domain_randomization_pipeline.py \
     --scene_path "/path/to/your/scene.usd" \
     --output_dir "./randomized_dataset" \
     --num_samples 2000
   ```

**Expected Outcome**: A diverse dataset with random variations in lighting, materials, and scene layouts.

**Learning Points**:
- Understanding domain randomization techniques
- Learning to implement various randomization in Isaac Sim
- Recognizing the importance of dataset diversity for model robustness

### Assignment 3: Perception Training Data Generation

**Objective**: Create a specialized pipeline for generating perception training data with accurate annotations.

**Detailed Steps**:
1. **Set Up Multi-Sensor Capture**:
   ```yaml
   # perception_data_config.yaml
   sensors:
     rgb_camera:
       resolution: [640, 480]
       fov: 90
       position: [0.1, 0.0, 1.2]  # On robot's head
       rotation: [0, 0, 0]

     depth_camera:
       resolution: [640, 480]
       fov: 90
       position: [0.1, 0.0, 1.2]
       rotation: [0, 0, 0]

     lidar:
       points: 64
       range: 25.0
       position: [0.15, 0.0, 1.0]
       rotation: [0, 0, 0]
   ```

2. **Create Annotation Pipeline**:
   ```python
   # annotation_pipeline.py
   import json
   import numpy as np
   from PIL import Image
   import os

   def create_annotation_file(frame_id, objects, save_path):
       """Create COCO-style annotation for the frame"""
       annotation = {
           "info": {
               "description": "Synthetic Humanoid Perception Dataset",
               "version": "1.0",
               "year": 2025
           },
           "licenses": [{"id": 1, "name": "MIT", "url": ""}],
           "images": [{
               "id": frame_id,
               "file_name": f"color/{frame_id:06d}.png",
               "height": 640,
               "width": 480,
               "date_captured": "2025-01-01"
           }],
           "annotations": []
       }

       # Add annotations for each object
       for i, obj in enumerate(objects):
           bbox = obj['bbox']  # [x, y, width, height]
           segmentation = obj['segmentation']

           annotation['annotations'].append({
               "id": len(annotation['annotations']) + 1,
               "image_id": frame_id,
               "category_id": obj['category_id'],
               "bbox": bbox,
               "area": bbox[2] * bbox[3],
               "iscrowd": 0,
               "segmentation": [segmentation],
               "pose": obj['pose']  # 6-DOF pose
           })

       # Save annotation file
       with open(os.path.join(save_path, f"annotations_{frame_id:06d}.json"), 'w') as f:
           json.dump(annotation, f)

   def process_frame_data(frame_data, output_dir, frame_id):
       """Process and save frame data with annotations"""
       # Save RGB image
       rgb_img = Image.fromarray(frame_data['rgb'])
       rgb_img.save(os.path.join(output_dir, 'color', f"{frame_id:06d}.png"))

       # Save depth image
       depth_img = Image.fromarray((frame_data['depth'] * 255).astype(np.uint8))
       depth_img.save(os.path.join(output_dir, 'depth', f"{frame_id:06d}.png"))

       # Create annotations
       create_annotation_file(frame_id, frame_data['objects'], os.path.join(output_dir, 'annotations'))
   ```

3. **Implement Dynamic Scene Generation**:
   ```python
   # dynamic_scene_generator.py
   import random
   import math

   def generate_dynamic_scene(scene_type="indoor_office"):
       """Generate a dynamic scene with moving objects"""
       scene_config = {
           "static_objects": [],
           "dynamic_objects": [],
           "lighting": {},
           "environment": {}
       }

       if scene_type == "indoor_office":
           # Add static office elements
           scene_config["static_objects"].extend([
               {"type": "desk", "position": [2, 0, 0], "rotation": [0, 0, 0]},
               {"type": "chair", "position": [2.2, 0.5, 0], "rotation": [0, 0, math.pi/2]},
               {"type": "bookshelf", "position": [-2, 1, 0], "rotation": [0, 0, 0]},
           ])

           # Add dynamic elements
           scene_config["dynamic_objects"].append({
               "type": "person",
               "start_pos": [-3, 0, 0],
               "end_pos": [3, 0, 0],
               "speed": random.uniform(0.5, 1.5),
               "path_type": "linear"
           })

           # Configure lighting
           scene_config["lighting"] = {
               "main_light": {"type": "distant", "intensity": 1500, "direction": [-0.5, -0.5, -1]},
               "fill_light": {"type": "dome", "intensity": 300}
           }

       return scene_config

   def apply_scene_config(stage, config):
       """Apply scene configuration to Isaac Sim stage"""
       # Implementation would depend on specific objects and their USD representations
       pass
   ```

4. **Create a Complete Generation Pipeline**:
   ```python
   # complete_perception_pipeline.py
   import os
   import random
   from tqdm import tqdm

   class PerceptionDataGenerator:
       def __init__(self, config_path):
           self.config = self.load_config(config_path)
           self.output_dir = self.config['output_dir']
           self.num_samples = self.config['num_samples']

           # Create output directories
           os.makedirs(os.path.join(self.output_dir, 'color'), exist_ok=True)
           os.makedirs(os.path.join(self.output_dir, 'depth'), exist_ok=True)
           os.makedirs(os.path.join(self.output_dir, 'segmentation'), exist_ok=True)
           os.makedirs(os.path.join(self.output_dir, 'annotations'), exist_ok=True)

       def load_config(self, config_path):
           """Load pipeline configuration"""
           with open(config_path, 'r') as f:
               return json.load(f)

       def generate_sample(self, sample_id):
           """Generate a single sample with all required data"""
           # Randomize scene
           scene_type = random.choice(["indoor_office", "outdoor_park", "warehouse"])
           scene_config = generate_dynamic_scene(scene_type)

           # Apply scene configuration in Isaac Sim
           apply_scene_config(omni.usd.get_context().get_stage(), scene_config)

           # Simulate briefly
           # (Implementation would use Isaac Sim's physics simulation)

           # Capture sensor data
           frame_data = self.capture_sensor_data()

           # Process and save the data
           process_frame_data(frame_data, self.output_dir, sample_id)

       def capture_sensor_data(self):
           """Capture data from all configured sensors"""
           # Implementation would interface with Isaac Sim's sensors
           return {
               "rgb": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),  # Placeholder
               "depth": np.random.random((480, 640)).astype(np.float32),  # Placeholder
               "objects": [
                   {
                       "category_id": random.randint(1, 10),
                       "bbox": [random.randint(0, 400), random.randint(0, 300), 100, 100],
                       "segmentation": [random.randint(0, 640) for _ in range(20)],  # Placeholder
                       "pose": {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}
                   }
               ]
           }

       def run_generation(self):
           """Run the complete generation pipeline"""
           print(f"Starting generation of {self.num_samples} samples...")

           for i in tqdm(range(self.num_samples), desc="Generating samples"):
               self.generate_sample(i)

           print(f"Generation complete! Dataset saved to {self.output_dir}")

   # Example usage
   if __name__ == "__main__":
       generator = PerceptionDataGenerator("config/perception_data_config.yaml")
       generator.run_generation()
   ```

5. **Execute with Training Integration**:
   ```bash
   # Generate perception training dataset
   python scripts/complete_perception_pipeline.py \
     --config config/perception_data_config.yaml \
     --output_dir ./perception_training_data \
     --num_samples 5000
   ```

**Expected Outcome**: A large, diverse dataset suitable for training perception models with accurate annotations in standard formats.

**Code Example for Training Integration**:
```python
# train_on_synthetic_data.py
import torch
import torchvision.transforms as transforms
from torch.utils.data import DataLoader, Dataset
import os
import json
from PIL import Image

class SyntheticPerceptionDataset(Dataset):
    def __init__(self, dataset_path, transform=None):
        self.dataset_path = dataset_path
        self.transform = transform

        # Load annotations
        self.annotations = []
        annotation_files = [f for f in os.listdir(os.path.join(dataset_path, 'annotations')) if f.endswith('.json')]

        for ann_file in annotation_files:
            with open(os.path.join(dataset_path, 'annotations', ann_file), 'r') as f:
                data = json.load(f)
                self.annotations.extend(data['annotations'])

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        annotation = self.annotations[idx]

        # Load image
        img_path = os.path.join(self.dataset_path, annotation['image']['file_name'])
        image = Image.open(img_path).convert('RGB')

        if self.transform:
            image = self.transform(image)

        # Extract object information
        bbox = torch.tensor(annotation['bbox'], dtype=torch.float32)
        category = torch.tensor(annotation['category_id'], dtype=torch.long)

        return image, bbox, category

# Create dataset and dataloader
transform = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

dataset = SyntheticPerceptionDataset('./perception_training_data', transform=transform)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

print(f"Dataset loaded with {len(dataset)} annotated objects")
```

**Learning Points**:
- Understanding how to structure synthetic data for model training
- Learning to create standardized annotations for various perception tasks
- Recognizing the relationship between synthetic data and model performance

## Troubleshooting Common Issues

### Data Generation Problems
- **Problem**: Dataset is too homogeneous?
  - **Solution**: Increase domain randomization parameters and scene diversity

### Annotation Issues
- **Problem**: Annotations are incorrect or missing?
  - **Solution**: Verify Isaac Sim extension configurations and run data validation scripts

### Performance Issues
- **Problem**: Generation rate is too slow?
  - **Solution**: Optimize scene complexity, use parallel processing, or adjust capture frequency