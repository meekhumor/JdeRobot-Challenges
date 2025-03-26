# Source Code

https://github.com/meekhumor/JdeRobot-Challenges/tree/main/react_challenge

# TaskBuddy App

A React and redux based task management application built for the **JdeRobot GSoC 2024** React Challenge. This app extends a basic task manager with advanced features like filtering, categories, priorities, search, drag-and-drop reordering, and due dates.

---

https://github.com/user-attachments/assets/d5c1ac86-94d3-4945-b523-c424224c3349

---

## Running the Application 
1. **Clone the Repository**:
   ```
   git clone https://github.com/meekhumor/JdeRobot-Challenges.git
   cd react_challenge
   ```

2. **Install Dependencies:**
   ```
   npm install
   ```

4. **Start the Application:**
   ```
   npm start
   ```


**Application is live at https://taskbuddy-ztf4.onrender.com/**

---

## Features
- **Task Creation**: Add tasks with title, priority, category, and due date.
- **Task Management**: Remove tasks and mark them as completed.
- **Filtering Tasks**: Filter by completion status (All, Completed, Incomplete).
- **Task Categories**: Assign tasks to categories (Personal, Work, Groceries) and filter by category.
- **Task Priority**: Set priority (High, Medium, Low) and sort tasks by priority.
- **Search**: Search tasks by title.
- **Drag and Drop**: Reorder tasks using drag-and-drop functionality.
- **Due Dates**: Set due dates with notifications for upcoming deadlines.

---

## Project Structure

```
├── react_challenge
    ├── public                         
    │   └── index.html                   # HTML entry point
    └── src               
        ├── index.js                     # JavaScript entry point
        ├── components                  
        │   ├── App.jsx                  # Main app component
        │   ├── common                 
        │   │   ├── CategoryBadge.jsx    # Displays task category
        │   │   ├── EmptyState.jsx       # Empty state UI
        │   │   ├── HelpGuide.jsx        # Interactive help modal
        │   │   ├── LocalStorage.jsx     # Local storage 
        │   │   ├── PriorityBadge.jsx    # Displays task priority
        │   │   └── SearchBar.jsx        # Search input component
        │   ├── layout                
        │   │   ├── Footer.jsx           # Footer 
        │   │   ├── Header.jsx           # Header 
        │   │   └── Layout.jsx           # Main layout 
        │   └── tasks                  
        │       ├── TaskCard.jsx         # Individual task display
        │       ├── TaskFilter.jsx       # Filter controls
        │       ├── TaskForm.jsx         # Task creation form
        │       └── TaskList.jsx         # Draggable task list
        ├── redux                       
        │   ├── store.js                 # Redux store 
        │   ├── actions                
        │   │   ├── categoryActions.jsx  # Category actions
        │   │   ├── filterActions.jsx    # Filter actions
        │   │   └── taskActions.jsx      # Task actions
        │   └── reducers               
        │       ├── categoryReducer.jsx  # Category state
        │       ├── filterReducer.jsx    # Filter state
        │       ├── index.jsx            # Root reducer
        │       └── taskReducer.jsx      # Task state
        └── styles
```


---

## How to Navigate and Interact
### 1. Adding a Task

- Enter a title (required).
- Expand "Advanced Options" to set:

  - **Priority:** Select High, Medium, or Low from the dropdown.
  - **Category:** Choose an existing category or add a new one via the "+" button.
  - **Due Date:** Pick a date using the date picker.

- Click **Add Task** to save.


### 2. Managing Tasks

- **Complete:** Click the checkbox next to a task to mark it as completed.
- **Remove:** Click the trash icon to delete a task.

### 3. Filtering Tasks

- **All**: Show all tasks.
- **Completed**: Show only completed tasks.
- **Incomplete**: Show only incomplete tasks.

Select a category from the category filter dropdown to view tasks in that category.

### 4. Searching Tasks

Type in the search bar to filter tasks by title (case-insensitive).

### 5. Reordering Tasks

Drag tasks in the list using the mouse and drop them to reorder.

### 6. Due Dates and Notifications

- Tasks with due dates within 3 days show a notification badge in the Header bell icon.
- Click the bell to view upcoming tasks with their due dates.
