# Exercises in 3D Geometry, Rotations, and Poses

## Part A: Written (Analytical) Exercises

### Exercise A1: Rotation Matrix Validation
**Goal**: Practice checking if a 3×3 matrix is a valid rotation.

1. Given a general 3×3 matrix M:

   M =  
   [  0.36   -0.48    0.80 ]  
   [  0.80    0.60   -0.00 ]  
   [ -0.48    0.64    0.60 ]  

   1. Check if Mᵀ * M = I.  
   2. Compute det(M).  
   3. Decide whether M is a valid rotation matrix (orthonormal with determinant +1).  
   4. Suppose you see small numerical deviations. How would you *force* M to be a proper rotation?

---

### Exercise A2: Rotation Composition in 2D
**Goal**: Understand how multiple rotations compose into a single rotation.

1. Write the standard 2D rotation matrix for an angle α:

   R(α) =  
   [ cosα   −sinα ]  
   [ sinα    cosα ]

2. Show that R(α) * R(β) = R(α + β).  
3. Numerically verify for α = π/6 and β = −π/2.  
4. *(Optional)* Evaluate det( R(α)*R(β) ) and interpret the result.

---

### Exercise A3: Converting Rotation Matrices to Euler Angles
**Goal**: Practice extracting Euler angles (Roll–Pitch–Yaw) from a 3D rotation matrix.

1. Suppose you have the matrix R:

   R =  
   [ 0   0   1 ]  
   [ 1   0   0 ]  
   [ 0   1   0 ]

2. Assume the convention Rz(γ) * Ry(β) * Rx(α). Find α, β, γ such that:  
   R = Rz(γ) * Ry(β) * Rx(α).

3. Are there multiple valid solutions? Explain briefly why Euler angles can be non-unique.

---

### Exercise A4: Pose Composition in 3D
**Goal**: Combine rotations and translations into a single transformation.

1. You have a pose T^A_B:

   T^A_B =  
   [ R^A_B   t^A_B ]  
   [  0ᵀ         1  ]  

   where  

   R^A_B =  
   [ 0   −1   0 ]  
   [ 1    0   0 ]  
   [ 0    0   1 ],  

   and  

   t^A_B =  
   [ 2 ]  
   [ 0 ]  
   [ 1 ].

2. Another pose T^B_C:

   R^B_C =  
   [ 1         0             0         ]  
   [ 0   cos(π/2)   −sin(π/2) ]  
   [ 0   sin(π/2)    cos(π/2)  ],  

   t^B_C =  
   [ 1 ]  
   [ 1 ]  
   [ 0 ].

3. Compute T^A_C = T^A_B * T^B_C. Find the final rotation R^A_C and translation t^A_C.  
4. Apply T^A_C to the point p^C = (1, 0, 0)ᵀ. Where does it land in frame A?

---

## Part B: Coding Exercises

### Exercise B1: Validate Random 3×3 Matrices
1. Implement a function `is_rotation_matrix(M)` that checks orthogonality and `det(M) ≈ +1`.  
2. Generate 10 random 3×3 matrices (entries in the range [−1,1]) and test them.  
3. *(Bonus)* Implement a re-orthonormalization step (e.g., via SVD or Gram–Schmidt) to fix nearly orthonormal matrices.

---

### Exercise B2: Euler Angles <-> Rotation Matrix
1. Implement `R_from_rpy(roll, pitch, yaw)` returning a 3×3 rotation matrix.  
2. Implement `rpy_from_R(R)` returning (roll, pitch, yaw).  
3. Randomly generate angles, convert to R, then convert back. Compare original vs. recovered angles.  
4. Note any discrepancies near `pitch ≈ ±90°`.

---

### Exercise B3: Axis–Angle and Quaternion Conversions
1. Write `axisAngle_to_quaternion(axis, theta)` using Rodrigues’ formula.  
2. Write `quaternion_to_axisAngle(q)` to invert it.  
3. Show for random axis–angle pairs that converting to quaternions and back yields the same geometry (modulo ± sign or 2π ambiguities).

---

### Exercise B4: Pose Composition and Point Transformation
1. Create a small class/struct to hold a 3D pose: a 3×3 rotation (or quaternion) + a 3D translation.  
2. Implement `compose(pose1, pose2)` that returns T^A_C = T^A_B * T^B_C.  
3. Implement `transformPoint(pose, pIn)` that applies T^A_B to p^B.  
4. Test with a known example (e.g., from Exercise A4).

---

## Key Points to Remember

- **Right-handed frames** and the condition that rotation matrices have `det(R) = +1`.
- **Euler angles**: minimal representation (3 parameters), can have singularities (gimbal lock), and order matters.
- **Axis–Angle (Rodrigues)**: intuitive for “rotate θ around axis u.”
- **Quaternions**: 4D, no singularities, but ±q represent the same rotation.
- **Poses** in 3D: combine rotation + translation into a 4×4 homogeneous transform. Composition and inverse are critical for multi-frame setups.