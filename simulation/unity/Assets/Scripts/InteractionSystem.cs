using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InteractionSystem : MonoBehaviour
{
    public Camera playerCamera;
    public float interactionDistance = 5f;
    public LayerMask interactionLayer;  // Set this to the layer your interactable objects are on

    private GameObject currentInteractable;
    private Outline currentOutline;

    void Start()
    {
        if (playerCamera == null)
        {
            playerCamera = Camera.main;
        }
        
        interactionLayer = LayerMask.GetMask("Interactable");
    }

    void Update()
    {
        HandleInteraction();
    }

    void HandleInteraction()
    {
        // Raycast to find interactable objects
        RaycastHit hit;
        if (Physics.Raycast(playerCamera.transform.position, playerCamera.transform.forward, out hit, interactionDistance, interactionLayer))
        {
            GameObject hitObject = hit.collider.gameObject;

            // If we hit a new interactable object, highlight it
            if (hitObject != currentInteractable)
            {
                RemoveHighlight();
                currentInteractable = hitObject;
                
                // Add highlight using Outline component (requires Unity Standard Assets or similar)
                currentOutline = hitObject.GetComponent<Outline>();
                if (currentOutline == null)
                {
                    currentOutline = hitObject.AddComponent<Outline>();
                    currentOutline.OutlineMode = Outline.Mode.OutlineAll;
                    currentOutline.OutlineColor = Color.yellow;
                    currentOutline.OutlineWidth = 3f;
                }
                else
                {
                    currentOutline.enabled = true;
                }
            }

            // Check for interaction input (e.g., 'E' key)
            if (Input.GetKeyDown(KeyCode.E))
            {
                InteractWithObject(hitObject);
            }
        }
        else
        {
            // If we're not hitting anything, remove highlight
            RemoveHighlight();
        }
    }

    void InteractWithObject(GameObject target)
    {
        Debug.Log("Interacting with: " + target.name);
        
        // Add interaction logic here based on the object type
        if (target.tag == "Robot")
        {
            // Robot-specific interaction
            RobotInteraction(target);
        }
        else if (target.tag == "Sensor")
        {
            // Sensor-specific interaction
            SensorInteraction(target);
        }
        else if (target.tag == "Environment")
        {
            // Environment-specific interaction
            EnvironmentInteraction(target);
        }
        else
        {
            // General interaction
            GeneralInteraction(target);
        }
    }

    void RobotInteraction(GameObject robot)
    {
        Debug.Log("Robot interaction: Controlling or inspecting the robot");
        // Add robot control or inspection logic here
    }

    void SensorInteraction(GameObject sensor)
    {
        Debug.Log("Sensor interaction: Activating or configuring sensor");
        // Add sensor activation/config logic here
    }

    void EnvironmentInteraction(GameObject environment)
    {
        Debug.Log("Environment interaction: Modifying environment");
        // Add environment modification logic here
    }

    void GeneralInteraction(GameObject obj)
    {
        Debug.Log("General interaction: " + obj.name);
        // Add general interaction logic here
    }

    void RemoveHighlight()
    {
        if (currentOutline != null)
        {
            currentOutline.enabled = false;
            currentInteractable = null;
        }
    }
}

// Simple Outline component for highlighting interactable objects
// This is a simplified version - in practice you'd likely use a more sophisticated approach
public class Outline : MonoBehaviour
{
    public enum Mode
    {
        OutlineAll,
        OutlineVisible,
        OutlineHidden,
        OutlineAndSilhouette,
        SilhouetteOnly
    }

    public Mode OutlineMode { get; set; }
    public Color OutlineColor { get; set; }
    public float OutlineWidth { get; set; }
    public bool enabled { get; set; }

    public Outline()
    {
        enabled = true;
        OutlineMode = Mode.OutlineAll;
        OutlineColor = Color.yellow;
        OutlineWidth = 2f;
    }
}