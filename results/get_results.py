import os


def read_matrix(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        matrix = []
        for line in lines:
            row = list(map(int, line.strip().split(' ')))
            matrix.append(row)
        return matrix


def compare_matrices(files):
    first_matrix = read_matrix(files[0])

    for file in files[1:]:
        current_matrix = read_matrix(file)
        if first_matrix != current_matrix:
            return False
    return True


def and_matrices(matrices):
    result = matrices[0]
    for matrix in matrices[1:]:
        for i, row in enumerate(matrix):
            for j, element in enumerate(row):
                result[i][j] &= element
    return result


def calculate_metrics(real, predicted):
    TP, FP, TN, FN = 0, 0, 0, 0

    for i, row in enumerate(real):
        for j, element in enumerate(row):
            if real[i][j] == 1 and predicted[i][j] == 1:
                TP += 1
            elif real[i][j] == 0 and predicted[i][j] == 1:
                FP += 1
            elif real[i][j] == 0 and predicted[i][j] == 0:
                TN += 1
            elif real[i][j] == 1 and predicted[i][j] == 0:
                FN += 1

    precision = TP / (TP + FP) if TP + FP > 0 else 0
    recall = TP / (TP + FN) if TP + FN > 0 else 0
    f1_score = 2 * precision * recall / \
        (precision + recall) if precision + recall > 0 else 0
    accuracy = (TP + TN) / (TP + FP + TN + FN) if TP + FP + TN + FN > 0 else 0

    return precision, recall, f1_score, accuracy


def calculate_metrics_for_directory(directory):
    file_prefix = 'real_'

    # Get a list of files that start with the given prefix
    files = [os.path.join(directory, f) for f in os.listdir(
        directory) if f.startswith(file_prefix) and f.endswith('.txt')]

    # Check if all matrices are the same
    if not compare_matrices(files):
        print("Not all real loop candidates matrices are the same in directory {}. Redo evaluations.".format(directory))
        return None

    real_loop_candidates = read_matrix(files[0])

    # Get a list of files that do not start with the given prefix
    files = [os.path.join(directory, f) for f in os.listdir(
        directory) if not f.startswith(file_prefix) and f.endswith('.txt')]

    # Read the matrices from the files
    matrices = [read_matrix(file) for file in files]

    # Perform elementwise AND operation on the matrices
    total_predicted_loop_candidates = and_matrices(matrices)

    # Calculate metrics
    precision, recall, f1_score, accuracy = calculate_metrics(
        real_loop_candidates, total_predicted_loop_candidates)

    total_method_name = " + ".join(
        [".".join(os.path.basename(file).split('.')[:-1]) for file in files])

    print(f"Calculated results for {directory}")
    print(f"Combined method: {total_method_name}")
    print("Precision: {:.3f}".format(precision))
    print("Recall: {:.3f}".format(recall))
    print("F1 Score: {:.3f}".format(f1_score))
    print("Accuracy: {:.3f}".format(accuracy))


if __name__ == "__main__":
    path = '.'  # Set the path to the folder containing the subdirectories

    # Get a list of directories in the current path
    subdirectories = [os.path.join(path, d) for d in os.listdir(
        path) if os.path.isdir(os.path.join(path, d))]

    for directory in subdirectories:
        calculate_metrics_for_directory(directory)
