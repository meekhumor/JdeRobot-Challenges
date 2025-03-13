export const formatDateShort = (date) => {
    if (!date) return '';
    const options = { month: 'short', day: 'numeric' };
    return new Date(date).toLocaleDateString(undefined, options);
  };
  
  export const formatDateLong = (date) => {
    if (!date) return '';
    const options = { month: 'long', day: 'numeric', year: 'numeric' };
    return new Date(date).toLocaleDateString(undefined, options);
  };
  
  export const isOverdue = (dueDate) => {
    if (!dueDate) return false;
    return new Date(dueDate) < new Date();
  };
  
  export const daysUntilDue = (dueDate) => {
    if (!dueDate) return null;
    const diffTime = new Date(dueDate) - new Date();
    return Math.ceil(diffTime / (1000 * 60 * 60 * 24));
  };