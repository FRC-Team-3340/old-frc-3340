import React, {Component} from "react";
import Carousel from 'react-bootstrap/Carousel';
import './showcase.css'

export class Showcase extends Component {
    constructor(props: any) {
        super(props);
    }

render()
{
    return (
        <>
            <Carousel className="showcase-carousel">
                <Carousel.Item>
                    <img
                        className="d-block w-100"
                        src="holder.js/800x400?text=Team 3340 Group Photo&bg=373940"
                        alt="Team 3340"
                    />
                    {/* Each image has a source (src) and alternate text when the mouse hovers over it. */}
                    <Carousel.Caption>
                        {/* Each carousel section has a header and caption */}
                        <h3>First slide label</h3>
                        <p>This is where recent stuff would go.</p>
                    </Carousel.Caption>
                </Carousel.Item>
                <Carousel.Item>
                    <img
                        className="d-block w-100"
                        src="holder.js/800x400?text=DUMMY&bg=282c34"
                        alt="Second slide"
                    />

                    <Carousel.Caption>
                        <h3>Second slide label</h3>
                        <p>Scrapbook thing.</p>
                    </Carousel.Caption>
                </Carousel.Item>
                <Carousel.Item>
                    <img
                        className="d-block w-100"
                        src="holder.js/800x400?text=DUMMY&bg=20232a"
                        alt="Third slide"
                    />

                    <Carousel.Caption>
                        <h3>Third slide label</h3>
                        <p>
                            Meet the team?
                        </p>
                    </Carousel.Caption>
                </Carousel.Item>
            </Carousel>
        </>
    );
}
}

export default Showcase;