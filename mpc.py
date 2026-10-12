# Standard math functions only (No external libraries like scipy or matplotlib)
import math

# --- MATRIX HELPERS FROM SCRATCH ---
# To avoid third-party libraries, we define standard matrix math routines.

def matrix_multiply(A, B):
    """Multiplies two matrices A and B."""
    return [[sum(a * b for a, b in zip(A_row, B_col)) for B_col in zip(*B)] for A_row in A]

def matrix_transpose(A):
    """Transposes matrix A."""
    return [list(x) for x in zip(*A)]

def matrix_add(A, B):
    """Adds two matrices of identical size."""
    return [[A[r][c] + B[r][c] for c in range(len(A[0]))] for r in range(len(A))]

def matrix_subtract(A, B):
    """Subtracts matrix B from matrix A."""
    return [[A[r][c] - B[r][c] for c in range(len(A[0]))] for r in range(len(A))]

def invert_matrix_2x2(M):
    """Inverts a 2x2 matrix analytically."""
    det = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(det) < 1e-9:
        raise ValueError("Matrix is singular and cannot be inverted.")
    inv_det = 1.0 / det
    return [
        [ M[1][1] * inv_det, -M[0][1] * inv_det],
        [-M[1][0] * inv_det,  M[0][0] * inv_det]
    ]


# --- SYSTEM & CONTROL VARIABLES ---

# System Dynamics (Discrete-time mass-spring-damper system)
# x_k+1 = A * x_k + B * u_k
A = [
    [1.0,  0.1],  # State 0: Position
    [-0.5, 0.9]   # State 1: Velocity
]
B = [
    [0.0],
    [0.1]
]

# Control Knobs (Weights)
# Since Np=2, we can explicitly compute matrices without complex loops
Q_pos = 10.0  # Weight tracking penalty for position
R_eff = 0.1   # Weight effort penalty for control input (u)

# Current State Tracker
x_current = [[2.0], [0.0]]  # Starts 2 meters away from target
target_pos = 0.0            # Bring system position to 0


# --- RECURSIVE MPC LOOP ---
# This loop computes the analytical predictive control law for Np=2 steps ahead.

print(f"{'Step':<6} | {'Current Position':<18} | {'Current Velocity':<18} | {'Control Action (u)':<18}")
print("-" * 70)

for step in range(15):
    # --- 1. Predictive Math (Unfolding the Horizon Np=2) ---
    # Prediction equations:
    # x_1 = A*x_0 + B*u_0
    # x_2 = A^2*x_0 + A*B*u_0 + B*u_1
    
    # We want to find an optimal u_0 that minimizes tracking errors at x_1 and x_2.
    # To keep the inversion clean, let's look at the optimal open-loop input solution 
    # for the next action based on minimizing: Error^T * Q * Error + u^T * R * u.
    
    # Derivation step: u_0 = - (G^T * Q_total * G + R_total)^-1 * G^T * Q_total * (F * x_current - Reference)
    # For a horizon step Np=1 to cleanly show 2x2 algebraic mapping without third-party frameworks:
    
    # Cost calculation mapped down to the immediate step:
    # We isolate the influence of input 'u' on position.
    # From A and B: next_position = 1.0 * position + 0.1 * velocity + 0.0 * u
    # Wait, the position doesn't change immediately because B[0][0]=0. But position changes at step 2!
    
    # Let's compute step 2 position directly:
    # position_2 = 1.0 * position_1 + 0.1 * velocity_1
    #            = 1.0 * (1.0*pos + 0.1*vel) + 0.1 * (-0.5*pos + 0.9*vel + 0.1*u_0)
    #            = 0.95 * pos + 0.19 * vel + 0.01 * u_0
    
    # Therefore, the sensitivity coefficient of u_0 on future position_2 is 0.01.
    # The natural drift of position_2 without input is (0.95 * pos + 0.19 * vel).
    
    pos = x_current[0][0]
    vel = x_current[1][0]
    
    drift_pos2 = 0.95 * pos + 0.19 * vel
    sensitivity = 0.01
    
    # Target position is 0. We want: sensitivity * u_0 + drift_pos2 = 0
    # Weighted optimization balance: 
    # derivative of cost w.r.t u_0: 2 * Q_pos * sensitivity * (sensitivity * u_0 + drift_pos2) + 2 * R_eff * u_0 = 0
    # Solve for u_0:
    
    numerator = -Q_pos * sensitivity * drift_pos2
    denominator = (Q_pos * (sensitivity ** 2)) + R_eff
    
    optimal_u0 = numerator / denominator
    
    # --- 2. Manual Constraint Saturation (The Rules) ---
    u_max = 2.0
    u_min = -2.0
    if optimal_u0 > u_max:
        optimal_u0 = u_max
    elif optimal_u0 < u_min:
        optimal_u0 = u_min
        
    # Print progress output to terminal safely
    print(f"{step:<6} | {pos:<18.4f} | {vel:<18.4f} | {optimal_u0:<18.4f}")
    
    # --- 3. Receding Horizon Rule ---
    # Apply ONLY the first action to the true system and advance time
    u_matrix = [[optimal_u0]]
    x_next = matrix_add(matrix_multiply(A, x_current), matrix_multiply(B, u_matrix))
    x_current = x_next
