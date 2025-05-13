import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from io import BytesIO
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# ---------------------- UI Setup ----------------------
st.set_page_config(page_title="SmartCoat Optimizer", layout="wide")
st.title("🧪 SmartCoat Optimizer Tool")
# 🎉 Welcome Box
st.markdown("""
<div style='background-color:#f0f2f6;padding:15px;border-radius:10px'>
    <h3 style='color:#0e1117'>Welcome to the SmartCoat Optimizer 👋</h3>
    <p style='color:#31333f;font-size:16px'>
        This tool helps you sequence coating jobs efficiently by minimizing changeovers and prioritizing urgent tasks. 
        You can manually enter jobs or upload a CSV, define your coating chemical types, and generate an optimized Gantt schedule — all in one place!
    </p>
</div>
""", unsafe_allow_html=True)

# 📘 Tutorial Box
    with st.expander("📘 How to Use This Tool (Click to expand)"):
    st.markdown(\"\"\"
    1. **Select how many chemical types** you're working with (C1, C2...).
    2. **Define changeover times** between each pair of chemicals.
    3. Choose between:
        - ✅ **Manual entry** (add each job one by one), or
        - 📄 **Upload a CSV** with job details
    4. Click **🚀 Optimize Schedule** to run the optimization.
    5. View the **Gantt chart**, and download the:
        - 📊 Optimized PNG chart
        - 📋 Job sequence as CSV
    \"\"\")

st.markdown("Define your **chemical changeover times** and **manually add coating jobs** or **upload CSV** to optimize scheduling.")

# ---------------------- Chemical Type Setup ----------------------
num_chemicals = st.number_input("How many chemical types?", min_value=2, max_value=10, value=3, step=1)
chemical_labels = [f"C{i+1}" for i in range(num_chemicals)]

st.subheader("🔄 Define Changeover Times (in minutes)")
changeover_inputs = {}
cols = st.columns(len(chemical_labels) + 1)

# Header row
with cols[0]:
    st.write("**From → To**")
for j, to_chem in enumerate(chemical_labels):
    with cols[j + 1]:
        st.write(f"**{to_chem}**")

# Input grid
for i, from_chem in enumerate(chemical_labels):
    row = st.columns(len(chemical_labels) + 1)
    with row[0]:
        st.write(f"**{from_chem}**")
    for j, to_chem in enumerate(chemical_labels):
        default = 0 if from_chem == to_chem else 15
        with row[j + 1]:
            value = st.number_input(label=f"{from_chem}→{to_chem}", key=f"{from_chem}_{to_chem}", value=default, min_value=0, max_value=999)
            changeover_inputs[(from_chem, to_chem)] = value

# ---------------------- Job Input Mode ----------------------
st.subheader("📝 Job Input Mode")
manual_mode = st.checkbox("Enter jobs manually instead of uploading a CSV", value=False)

if "manual_jobs" not in st.session_state:
    st.session_state.manual_jobs = []

job_df = None

if manual_mode:
    st.markdown("### ➕ Add New Coating Job")
    with st.form("job_entry_form", clear_on_submit=True):
        col1, col2, col3 = st.columns(3)
        with col1:
            job_id = st.text_input("Job ID", "")
            slide_type = st.text_input("Slide Type", "Type1")
        with col2:
            chemical_type = st.selectbox("Chemical Type", options=chemical_labels)
            priority = st.selectbox("Priority", options=[1, 2, 3])
        with col3:
            duration = st.number_input("Estimated Time (min)", min_value=1, max_value=999, value=30)

        submitted = st.form_submit_button("➕ Add Job")

    if submitted and job_id:
        st.session_state.manual_jobs.append({
            "Job_ID": job_id,
            "Chemical_Type": chemical_type,
            "Slide_Type": slide_type,
            "Priority": priority,
            "Estimated_Time_mins": duration
        })

    if st.session_state.manual_jobs:
        job_df = pd.DataFrame(st.session_state.manual_jobs)
        st.success("✅ Job list built!")
        st.dataframe(job_df)
else:
    uploaded_file = st.file_uploader("Upload your job list", type="csv")
    if uploaded_file:
        job_df = pd.read_csv(uploaded_file)
        st.success("✅ Job data loaded from CSV!")
        st.dataframe(job_df)

# ---------------------- Optimization Functions ----------------------
def calculate_changeover_matrix(df, changeover_map):
    matrix = pd.DataFrame(0, index=df["Job_ID"], columns=df["Job_ID"])
    for i in range(len(df)):
        for j in range(len(df)):
            if i == j:
                continue
            from_chem = df.iloc[i]["Chemical_Type"]
            to_chem = df.iloc[j]["Chemical_Type"]
            matrix.iloc[i, j] = changeover_map.get((from_chem, to_chem), 999)
    return matrix

def build_cost_matrix(df, changeover_df):
    n = len(df)
    cost_matrix = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i == j:
                cost = 0
            else:
                duration = df.iloc[j]["Estimated_Time_mins"]
                changeover = changeover_df.iloc[i, j]
                priority = df.iloc[j]["Priority"]
                priority_weight = 4 - priority  # P1=3, P2=2, P3=1
                cost = int((duration + changeover) / priority_weight)
            cost_matrix[i][j] = cost
    return cost_matrix

def solve_job_sequence(cost_matrix, df):
    n = len(cost_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def cost_callback(from_idx, to_idx):
        return cost_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]

    transit_callback_index = routing.RegisterTransitCallback(cost_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_params)

    if solution:
        index = routing.Start(0)
        route = []
        total_time = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(df.iloc[node]["Job_ID"])
            prev_index = index
            index = solution.Value(routing.NextVar(index))
            total_time += routing.GetArcCostForVehicle(prev_index, index, 0)
        return route, total_time
    else:
        return None, None

# [Rest of the code remains unchanged]
