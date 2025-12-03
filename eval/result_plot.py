import matplotlib.pyplot as plt
import numpy as np

# ================= DATA =================
objects = ['Table & Chair', 'Cone', 'Cardboard Box']

# Accuracy (Lower is Better)
# Values from your experiments
acc_proposal = [2.63, 2.48, 4.15]
acc_baseline = [5.50, 4.73, 5.52]

# Completeness (Lower is Better)
comp_proposal = [8.59, 4.18, 4.39]
comp_baseline = [2.47, 1.45, 3.33]

# Efficiency (File Size in MB)
size_proposal = 1.7
size_baseline = 169.0
# ========================================

def plot_metric(metric_name, data_prop, data_base, ylabel):
    x = np.arange(len(objects))
    width = 0.35

    fig, ax = plt.subplots(figsize=(8, 5))
    rects1 = ax.bar(x - width/2, data_prop, width, label='Proposal (OTSLAM)', color='#ffb000') # Yellow/Orange
    rects2 = ax.bar(x + width/2, data_base, width, label='Baseline (RTAB-Map)', color='#0065bd') # Blue

    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(f'{metric_name} Comparison (Lower is Better)', fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(objects, fontsize=11)
    ax.legend()
    ax.grid(axis='y', linestyle='--', alpha=0.7)

    # Add labels on top
    def autolabel(rects):
        for rect in rects:
            height = rect.get_height()
            ax.annotate(f'{height:.2f}',
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')

    autolabel(rects1)
    autolabel(rects2)
    
    plt.tight_layout()
    plt.savefig(f'result_{metric_name.lower()}.png', dpi=300)
    print(f"✅ Saved chart: result_{metric_name.lower()}.png")
    plt.show()

# 1. Plot Accuracy
plot_metric('Accuracy', acc_proposal, acc_baseline, 'Mean Error (cm)')

# 2. Plot Completeness
plot_metric('Completeness', comp_proposal, comp_baseline, 'Mean Distance to Model (cm)')

# 3. Plot Efficiency (Log Scale due to massive difference)
fig, ax = plt.subplots(figsize=(6, 5))
langs = ['Proposal', 'Baseline']
sizes = [size_proposal, size_baseline]
bars = ax.bar(langs, sizes, color=['#ffb000', '#0065bd'])
ax.set_ylabel('Map File Size (MB)', fontsize=12)
ax.set_title('Storage Efficiency (Lower is Better)', fontsize=14, fontweight='bold')
ax.bar_label(bars, fmt='%.1f MB')
plt.tight_layout()
plt.savefig('result_efficiency.png', dpi=300)
print("✅ Saved chart: result_efficiency.png")
plt.show()