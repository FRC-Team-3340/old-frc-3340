import React from 'react'
import ReactDOM from 'react-dom/client'
import App from './App'
import MenuBar from "./components/navbar";
import './index.css'
import Showcase from "./components/showcase";

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
    <React.StrictMode>
        <MenuBar />
        <Showcase/>
        <App/>
    </React.StrictMode>,
)
