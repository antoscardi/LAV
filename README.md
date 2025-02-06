# Localization of Avalanche Victims by a Team of UAVs 

 ![page_1_presentation](https://github.com/user-attachments/assets/e32742d9-2236-47d0-baa6-568141ca5f67)


## 📌 Overview  

This repository contains my **thesis work** on **decentralized UAV swarm control** for **multi-source localization of avalanche victims**. The approach leverages the **Particle Swarm Optimization (PSO)** and **Recursive Least Squares (RLS)** algorithms to estimate the victims position using electromagnetic signals (**ARTVA** sensor).  

### **Key Highlights**  
- 🛰 **Swarm Intelligence** applied to rescue operations  
- 📡 **Electromagnetic signal processing** for localization
- 🚁 **Drone flight autonomous control**
- ⚡ **PSO & RLS optimization algorithms**  
- 🔍 **Decentralized control strategy**  

---

## 🎥 Simulations  
  
### 🔹Exploration Phase:
   
![quadrotor_simulation](https://github.com/user-attachments/assets/647e28a1-1fc7-40cd-b85e-7c2077849297)


### 🔹 Exploitation Phase:
  
![case_1](https://github.com/user-attachments/assets/97c2b553-4883-4778-825e-7233b651fcd3)


---

## 📜 Thesis Details  

- 🎓 **Institution:** La Sapienza University of Rome  
- 📅 **Year:** 2025  
- 👨‍🏫 **Supervisor:** Andrea Cristofaro 
- 📄 **Thesis Document:** [Download PDF](LaTeX/main.pdf)  

---

### **📌 How to Run the Code**  

#### 👤 Multiple Victims Case  
```bash
git clone https://github.com/your-repo/thesis.git
cd PSO_superpos
matlab -r "main"
  ```

#### 👥 Single Victims Case 
```bash
cd ../RLS_indipendent && matlab -r "main"
  ```
