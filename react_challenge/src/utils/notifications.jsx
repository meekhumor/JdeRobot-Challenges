export const requestNotificationPermission = () => {
    if (!('Notification' in window)) {
      console.log('This browser does not support notifications');
      return;
    }
  
    if (Notification.permission !== 'granted' && Notification.permission !== 'denied') {
      Notification.requestPermission();
    }
  };
  
  export const showNotification = (title, options = {}) => {
    if (Notification.permission === 'granted') {
      new Notification(title, {
        body: options.body || 'Task reminder',
        icon: '/assets/logo.svg', // Adjust path as needed
        ...options,
      });
    }
  };
  
  // Example usage for upcoming tasks
  export const notifyUpcomingTasks = (tasks) => {
    const upcomingTasks = tasks.filter(task => {
      if (!task.dueDate || task.completed) return false;
      const dueDate = new Date(task.dueDate);
      const now = new Date();
      const diffTime = dueDate - now;
      const diffDays = Math.ceil(diffTime / (1000 * 60 * 60 * 24));
      return diffDays >= 0 && diffDays <= 1; // Notify for tasks due within 1 day
    });
  
    upcomingTasks.forEach(task => {
      showNotification(`Task Due Soon: ${task.title}`, {
        body: `Due on ${new Date(task.dueDate).toLocaleDateString()}`,
      });
    });
  };