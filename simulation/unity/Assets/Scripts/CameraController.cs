using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float moveSpeed = 10f;
    public float lookSpeed = 2f;
    public float scrollSpeed = 20f;
    
    private Camera mainCamera;
    
    void Start()
    {
        mainCamera = GetComponent<Camera>();
        if (mainCamera == null)
        {
            mainCamera = Camera.main;
        }
    }

    void Update()
    {
        HandleMovement();
        HandleRotation();
        HandleZoom();
    }
    
    void HandleMovement()
    {
        float horizontal = 0f;
        float vertical = 0f;
        
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            vertical = 1f;
        }
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
        {
            vertical = -1f;
        }
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
        {
            horizontal = -1f;
        }
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
        {
            horizontal = 1f;
        }
        
        Vector3 movement = new Vector3(horizontal, 0, vertical) * moveSpeed * Time.deltaTime;
        transform.Translate(movement, Space.Self);
    }
    
    void HandleRotation()
    {
        if (Input.GetMouseButton(1)) // Right mouse button
        {
            float mouseX = Input.GetAxis("Mouse X") * lookSpeed;
            float mouseY = Input.GetAxis("Mouse Y") * lookSpeed;
            
            transform.Rotate(-mouseY, mouseX, 0, Space.Self);
            
            // Limit vertical rotation
            Vector3 rotation = transform.localEulerAngles;
            rotation.x = Mathf.Clamp(rotation.x, -85, 85);
            transform.localEulerAngles = rotation;
        }
    }
    
    void HandleZoom()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        transform.Translate(0, 0, scroll * scrollSpeed * Time.deltaTime, Space.Self);
        
        // Limit how far we can zoom
        Vector3 currentPos = transform.position;
        float distanceToOrigin = Vector3.Distance(currentPos, Vector3.zero);
        if (distanceToOrigin > 20f && scroll < 0)
        {
            transform.Translate(0, 0, -scroll * scrollSpeed * Time.deltaTime, Space.Self);
        }
        if (Vector3.Distance(transform.position, new Vector3(0, 1.5f, 0)) < 1f && scroll > 0)
        {
            transform.Translate(0, 0, -scroll * scrollSpeed * Time.deltaTime, Space.Self);
        }
    }
}