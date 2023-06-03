import matplotlib.pyplot as plt
import numpy as np
from sklearn.metrics import auc
import sys
import os

method_name = sys.argv[1] # The method name
result_path = sys.argv[2] # The path to the results file
# Read precisions and recalls from the input arguments
precisions = np.fromstring(sys.argv[3], sep=',')
recalls = np.fromstring(sys.argv[4], sep=',')

# Calculate AUPRC
auprc = auc(recalls, precisions)
print(f'AUPRC for {method_name}: {auprc}')

# Create a new figure
plt.figure()

# Plot the precision-recall curve
plt.plot(recalls, precisions, marker='.')

# Set the labels and title
plt.xlabel('Recall')
plt.ylabel('Precision')
plt.title('Precision-Recall Curve for ' + method_name)

# Display the average precision on the plot
plt.figtext(0.5, 0.5, 'AUPRC: {:.3f}'.format(auprc), ha='center', va='center')


# Save the figure
full_path = os.path.join(result_path, method_name + '_curve.png')
plt.savefig(full_path) 
