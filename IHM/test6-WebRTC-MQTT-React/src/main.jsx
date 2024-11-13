import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './teleop-interface.css'
import App from './App.jsx'
import './fontAwesome.jsx'

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
