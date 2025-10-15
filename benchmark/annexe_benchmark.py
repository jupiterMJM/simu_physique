"""
this file is an annexe file to the benchmark.py file
it contains some functions that can be useful to make more advanced benchmarks
:author: Maxence Barré
:date: 2025
"""

import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import time
import hashlib
from tqdm import tqdm

def test_computer_speed():
    """
    this function will run a few test to estimate the speed of the computer
    the test will be:
    - matrix multiplication of two large matrices
    - hash of a large string
    - sorting a large array
    - doing some euclidian division (get the rest of a division)
    :note: the aim is to do kind of a calibration of the computer, so that we can compare the speed of different computers
        plus, on the same computer but under differents charges
    """
    retour = {}

    # Test 1: Matrix multiplication
    size = 1000  # Size of the matrix
    np.random.seed(42)  # Set seed for reproducibility
    start_time = time.time()
    N_matrix = 100
    for _ in tqdm(range(N_matrix)):
        matrix_a = np.random.rand(size, size)
        matrix_b = np.random.rand(size, size)
        
        
        _ = np.dot(matrix_a, matrix_b)  # Perform matrix multiplication
    end_time = time.time()
        
    elapsed_time = end_time - start_time
    retour['matrix_multiplication_time'] = elapsed_time/N_matrix
    print(f"Matrix multiplication of size {size}x{size} took {elapsed_time:.4f} seconds.")

    # Test 2: Hashing a large string
    large_string =  ''.join(np.random.choice(list('abcdefghijklmnopqrstuvwxyz'), size=10**7))
    start_time = time.time()
    N_hash = 1000
    for _ in tqdm(range(N_hash)):
        _ = hashlib.sha256(large_string.encode()).hexdigest()  # Perform hashing
    end_time = time.time()
    elapsed_time = end_time - start_time
    retour['hashing_time'] = elapsed_time/N_hash
    print(f"Hashing a large string 100 times took {elapsed_time:.4f} seconds.")

    # Test 3: Sorting a large array
    large_array = np.random.rand(10**7)
    start_time = time.time()
    N_sort = 5
    for _ in tqdm(range(N_sort)):
        _ = np.sort(large_array)  # Perform sorting
    end_time = time.time()
    elapsed_time = end_time - start_time
    retour['sorting_time'] = elapsed_time/N_sort
    print(f"Sorting an array of size {len(large_array)} took {elapsed_time:.4f} seconds.")

    # Test 4: Euclidean division
    large_numbers = np.random.randint(1, 10**9, size=10**7)
    start_time = time.time()
    N_div = 100
    for _ in tqdm(range(N_div)):
        _ = large_numbers % 7  # Perform Euclidean division
    end_time = time.time()
    elapsed_time = end_time - start_time
    retour['euclidean_division_time'] = elapsed_time/N_div
    print(f"Euclidean division of {len(large_numbers)} numbers took {elapsed_time:.4f} seconds.")
    # score coefficient are taken arbitrarily
    score = "10*retour['matrix_multiplication_time'] + 50*retour['hashing_time'] + retour['sorting_time'] + 10*retour['euclidean_division_time']"
    retour["score_formula"] = score
    retour["score"] = eval(score)
    return retour


if __name__ == "__main__":
    print(test_computer_speed())