import React from 'react'
import {Navbar, Nav, NavDropdown, Container, Collapse} from 'react-bootstrap'
import 'bootstrap/dist/css/bootstrap.min.css'
import './navbar.css'

function MenuBar() {
    return (
        <Navbar className = 'navigation-bar' expand="lg" variant='dark'>
            <Container>
                <Navbar.Brand href="#home">MagneGeeks</Navbar.Brand>
                <Navbar.Toggle aria-controls="basic-navbar-nav"/>
                <Navbar.Collapse id="basic-navbar-nav">
                    <Nav className="me-auto">
                        <Nav.Link href="#home">Home</Nav.Link>
                        <Nav.Link href="#link">Link</Nav.Link>
                        <NavDropdown title="About" id="basic-nav-dropdown">
                            <NavDropdown.Item href="#action/3.1">About Us</NavDropdown.Item>
                            <NavDropdown.Item href="#action/3.2">
                                About FRC
                            </NavDropdown.Item>
                            <NavDropdown.Item href="#action/3.3">Meet the Team</NavDropdown.Item>
                            <NavDropdown.Divider/>
                            <NavDropdown.Item href="#action/3.4">
                                Separated link
                            </NavDropdown.Item>
                        </NavDropdown>
                    </Nav>
                </Navbar.Collapse>
            </Container>
        </Navbar>

    )
}

export default MenuBar