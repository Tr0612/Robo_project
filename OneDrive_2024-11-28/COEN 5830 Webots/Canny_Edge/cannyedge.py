import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
from collections import deque

# Canny Edge Detection Implementation
def canny_edge_detection(image, low_threshold, high_threshold):
    """Custom implementation of Canny Edge Detection."""

    # Step 1: Grayscale conversion
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Step 2: Gaussian smoothing
    smoothed = gaussian_filter(gray, sigma=1.4)

    # Step 3: Compute gradients using Sobel operator
    sobel_x = cv2.Sobel(smoothed, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(smoothed, cv2.CV_64F, 0, 1, ksize=3)
    gradient_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)
    gradient_direction = np.arctan2(sobel_y, sobel_x)

    # Step 4: Non-Maximum Suppression
    nms = non_maximum_suppression(gradient_magnitude, gradient_direction)

    # Step 5: Hysteresis Thresholding using DFS
    edges = hysteresis_with_dfs(nms, low_threshold, high_threshold)

    return edges

def non_maximum_suppression(gradient_magnitude, gradient_direction):
    """Perform Non-Maximum Suppression on gradient magnitudes."""
    M, N = gradient_magnitude.shape
    suppressed = np.zeros((M, N), dtype=np.float32)
    angle = gradient_direction * 180.0 / np.pi
    angle[angle < 0] += 180

    for i in range(1, M - 1):
        for j in range(1, N - 1):
            try:
                q = 255
                r = 255

                # Angle 0
                if (0 <= angle[i, j] < 22.5) or (157.5 <= angle[i, j] <= 180):
                    q = gradient_magnitude[i, j + 1]
                    r = gradient_magnitude[i, j - 1]
                # Angle 45
                elif 22.5 <= angle[i, j] < 67.5:
                    q = gradient_magnitude[i + 1, j - 1]
                    r = gradient_magnitude[i - 1, j + 1]
                # Angle 90
                elif 67.5 <= angle[i, j] < 112.5:
                    q = gradient_magnitude[i + 1, j]
                    r = gradient_magnitude[i - 1, j]
                # Angle 135
                elif 112.5 <= angle[i, j] < 157.5:
                    q = gradient_magnitude[i - 1, j - 1]
                    r = gradient_magnitude[i + 1, j + 1]

                if gradient_magnitude[i, j] >= q and gradient_magnitude[i, j] >= r:
                    suppressed[i, j] = gradient_magnitude[i, j]
                else:
                    suppressed[i, j] = 0

            except IndexError as e:
                pass

    return suppressed

def hysteresis_with_dfs(image, low_threshold, high_threshold):
    """Perform Hysteresis Thresholding using Depth-First Search (DFS)."""
    M, N = image.shape
    edges = np.zeros((M, N), dtype=np.uint8)

    # Mark strong edges
    strong_edges = (image >= high_threshold)
    weak_edges = ((image >= low_threshold) & (image < high_threshold))

    # Use DFS to link weak edges connected to strong edges
    def dfs(i, j):
        stack = deque([(i, j)])
        while stack:
            x, y = stack.pop()
            if edges[x, y] == 0:
                edges[x, y] = 255
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < M and 0 <= ny < N and weak_edges[nx, ny] and edges[nx, ny] == 0:
                            stack.append((nx, ny))

    for i in range(M):
        for j in range(N):
            if strong_edges[i, j] and edges[i, j] == 0:
                dfs(i, j)

    return edges

# Test the custom Canny Edge Detection
image = cv2.imread("test_img.png")
custom_edges = canny_edge_detection(image, low_threshold=50, high_threshold=150)

# Compare with OpenCV's implementation
opencv_edges = cv2.Canny(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 50, 150)

# Visualization
plt.figure(figsize=(15, 5))
plt.subplot(1, 3, 1)
plt.title("Original Image")
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.axis("off")

plt.subplot(1, 3, 2)
plt.title("Custom Canny")
plt.imshow(custom_edges, cmap="gray")
plt.axis("off")

plt.subplot(1, 3, 3)
plt.title("OpenCV Canny")
plt.imshow(opencv_edges, cmap="gray")
plt.axis("off")

plt.tight_layout()
plt.show()
