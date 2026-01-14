import sys
print("1. Python is running...")

try:
    import numpy as np
    print("2. NumPy imported successfully.")
except ImportError as e:
    print(f"FATAL: NumPy failed to import: {e}")
    sys.exit(1)

print("3. Testing Math Engine...")
try:
    # A simple dot product. If this crashes, your installation is corrupted.
    a = np.random.rand(100, 100)
    b = np.random.rand(100, 100)
    c = np.dot(a, b)
    print(f"4. Math successful. Result shape: {c.shape}")
except Exception as e:
    print(f"FATAL: Math Error: {e}")

print("5. Test Complete.")